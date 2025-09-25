function run_uav_vrep_static_target_with_cam
clc; close all; clearvars;

%% ===== 0) CoppeliaSim 安装路径（按需修改） =====
CS_ROOT = 'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu';

%% ===== 1) remApi 绑定 & DLL 处理（兼容不同目录结构） =====
bindDir = fullfile(CS_ROOT,'programming','legacyRemoteApi','remoteApiBindings','matlab','matlab');
addpath(bindDir);

dllSearch = dir(fullfile(CS_ROOT,'programming','legacyRemoteApi','remoteApiBindings','**','remoteApi.dll'));
assert(~isempty(dllSearch),'未找到 remoteApi.dll：请确认安装完整并搜索路径正确');
dllDir  = dllSearch(1).folder;
dllFull = fullfile(dllDir,'remoteApi.dll');
setenv('PATH', [getenv('PATH') ';' dllDir]);  % 让进程可找到依赖 DLL

if libisloaded('remoteApi'); unloadlibrary('remoteApi'); end
loadlibrary(dllFull,@remoteApiProto);          % 手动加载 DLL，避免 “找不到指定的模块”

vrep = remApi('remoteApi');                    % 面向对象封装
vrep.simxFinish(-1);                           % 关闭残留连接

%% ===== 2) 连接仿真（19997 端口，需在 remoteApiConnections.txt 开启） =====
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
assert(clientID>-1,'连接失败：请重启 CoppeliaSim 并检查 remoteApiConnections.txt 已启用 19997 端口');

% 打通通信并稍等
vrep.simxGetPingTime(clientID);
pause(0.1);

%% ===== 3) 句柄：机体 / 目标 / 机载相机（稳健版） =====
[quadH, quadName] = getHandleRobust(vrep, clientID, {'Quadcopter','Quadricopter'}, 'quadcopter');
[tgtH,  tgtName ] = getHandleRobust(vrep, clientID, {'target','Target','Quadcopter_target','Quadricopter_target'}, 'target');
[camH,  camName ] = getHandleRobust(vrep, clientID, {'visionSensor','Vision_sensor','frontCamera','Quadricopter_frontCamera'}, 'vision');

fprintf('[INFO] 使用对象: quad=%s | target=%s | cam=%s\n', quadName, tgtName, camName);

% ===== 不在客户端设置 sim_* 参数（如 active/分辨率） =====
% 若确需改分辨率，可使用参数 ID 数值（示例，通常不必）：
% paramVisionResX = 1000;  % sim_visionintparam_resolution_x
% paramVisionResY = 1001;  % sim_visionintparam_resolution_y
% vrep.simxSetObjectIntParameter(clientID, camH, paramVisionResX, 320, vrep.simx_opmode_oneshot);
% vrep.simxSetObjectIntParameter(clientID, camH, paramVisionResY, 240, vrep.simx_opmode_oneshot);

%% ===== 4) 同步模式并启动仿真 =====
vrep.simxSynchronous(clientID,true);
startRC = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
assert(startRC==0,'启动仿真失败（返回码=%d）',startRC);

%% ===== 5) 目标固定到世界坐标（静止） =====
target_pos_world = [0.0, 0.0, 1.5];     % [x y z]，可修改
target_yaw       = 0.0;                 % yaw
vrep.simxSetObjectPosition   (clientID, tgtH, -1, target_pos_world, vrep.simx_opmode_oneshot);
vrep.simxSetObjectOrientation(clientID, tgtH, -1, [0 0 target_yaw], vrep.simx_opmode_oneshot);

%% ===== 6) 传感与图像流预热 =====
% 机体位姿（stream 预热）
vrep.simxGetObjectPosition   (clientID, quadH, -1, vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID, quadH, -1, vrep.simx_opmode_streaming);

% 检查是否支持新版图像 API
haveImg2 = ismethod(vrep,'simxGetVisionSensorImage2');
if haveImg2
    fprintf('图像API: 新版 simxGetVisionSensorImage2\n');
else
    fprintf('图像API: 旧版 simxGetVisionSensorImage\n');
end

% 图像窗口
hFig = figure('Color','w','Name','UAV Front Camera', 'NumberTitle', 'off'); 
hAx  = axes('Parent',hFig); 
hIm  = imshow(zeros(240,320,3,'uint8'), 'Parent',hAx); 
title(hAx,'Camera (live)');

% 图像流预热 - streaming 获取数帧
warmupN = 12;
for i = 1:warmupN
    if haveImg2
        [~, ~] = vrep.simxGetVisionSensorImage2(clientID, camH, 0, vrep.simx_opmode_streaming);
    else
        [~, ~, ~] = vrep.simxGetVisionSensorImage(clientID, camH, 0, vrep.simx_opmode_streaming);
    end
    vrep.simxSynchronousTrigger(clientID);
    pause(0.03);
end

%% ===== 6) 主循环：只推进仿真 & 拉相机画面（目标静止不动） =====
DT = 0.05; T_total = 30; N = round(T_total/DT);
imgCount = 0;

for k = 1:N
    % 读取相机图像
    if haveImg2
        [rcI, img] = vrep.simxGetVisionSensorImage2(clientID, camH, 0, vrep.simx_opmode_buffer);
        if rcI == 0 && ~isempty(img)
            set(hIm, 'CData', img);
            imgCount = imgCount + 1;
            % 添加调试信息
            if mod(k, 10) == 0
                fprintf('图像尺寸: %dx%dx%d, 数据类型: %s\n', ...
                    size(img,1), size(img,2), size(img,3), class(img));
                fprintf('图像值范围: [%d, %d]\n', min(img(:)), max(img(:)));
            end
        else
            fprintf('获取图像失败 (代码: %d)\n', rcI);
            % 尝试重新启用相机
            if rcI == 1 % 可能表示相机未就绪
                vrep.simxSetObjectIntParameter(clientID, camH, 1002, 1, vrep.simx_opmode_oneshot);
            end
        end
    else
        % 兼容旧版API
        [rcI, res, img1D] = vrep.simxGetVisionSensorImage(clientID, camH, 0, vrep.simx_opmode_buffer);
        if rcI == 0 && ~isempty(img1D)
            % 转换图像数据
            imgU8 = uint8(img1D);
            w = res(1); h = res(2);
            
            % 重塑为3D数组 (高度 x 宽度 x 3)
            img3 = reshape(imgU8, [3, w, h]);
            img3 = permute(img3, [3, 2, 1]);
            img3 = flipud(img3); % 垂直翻转
            
            set(hIm, 'CData', img3);
            imgCount = imgCount + 1;
            
            % 添加调试信息
            if mod(k, 10) == 0
                fprintf('图像尺寸: %dx%dx%d, 数据类型: %s\n', ...
                    size(img3,1), size(img3,2), size(img3,3), class(img3));
                fprintf('图像值范围: [%d, %d]\n', min(img3(:)), max(img3(:)));
            end
        else
            fprintf('获取图像失败 (代码: %d)\n', rcI);
        end
    end
    
    % 更新标题显示帧率
    if mod(k, 20) == 0 && ishghandle(hFig)
        fps = imgCount / (k * DT);
        title(hAx, sprintf('Camera (live) - %.1f FPS', fps));
    end

    vrep.simxSynchronousTrigger(clientID); % 同步推进一步
    if ishghandle(hFig)
        drawnow limitrate;
    else
        break; % 如果窗口关闭，退出循环
    end
end%% ===== 8) 收尾（由 onCleanup 保证，即使异常也执行） =====
disp('✅ 目标静止 & 相机画面显示结束');

end  % ===== 主函数结束 =====


%% ------------------------------------------------------------------------
function [h, usedName] = getHandleRobust(vrep, clientID, nameList, fuzzyKey)
% 稳健句柄获取：
% 1) 先对常见名（含 #0/#1/#2 后缀）做精确尝试
% 2) 失败则枚举所有对象名，做一次包含 fuzzyKey 的模糊匹配
h = -1; usedName = '';
for i = 1:numel(nameList)
    tries = [{nameList{i}}, strcat(nameList{i},'#0'), strcat(nameList{i},'#1'), strcat(nameList{i},'#2')];
    for t = 1:numel(tries)
        [rc, hh] = vrep.simxGetObjectHandle(clientID, tries{t}, vrep.simx_opmode_blocking);
        if rc==0
            h = hh; usedName = tries{t};
            fprintf('[OK] 命中对象名: %s\n', usedName);
            return;
        end
    end
end

[rcAll, handles] = vrep.simxGetObjects(clientID, vrep.sim_object_type_all, vrep.simx_opmode_blocking);
if rcAll==0
    for k = 1:numel(handles)
        [rcN, nm] = vrep.simxGetObjectName(clientID, handles(k), vrep.simx_opmode_blocking);
        if rcN==0 && contains(lower(nm), lower(fuzzyKey))
            h = handles(k); usedName = nm;
            fprintf('[OK] 回退匹配: %s\n', usedName);
            return;
        end
    end
end
error('未找到包含关键字 "%s" 的对象，请在场景中重命名为唯一名称（如 visionSensor / target）。', fuzzyKey);
end

%% ------------------------------------------------------------------------
function localCleanup(vrep, clientID)
% 安全收尾：停止仿真、断开连接、释放 DLL
try
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
catch, end
try
    vrep.simxFinish(clientID);
catch, end
try
    vrep.delete();
catch, end
try
    if libisloaded('remoteApi'); unloadlibrary('remoteApi'); end
catch, end
end
