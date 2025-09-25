function uav_visual_servo_impact_demo3d
clc; close all; clearvars;

%% Parameters
% Simulation
T_total    = 45.0;           % [s]
DT         = 0.02;           % [s]
REALTIME   = true;           % play roughly in real-time
N          = round(T_total/DT);

% 目标运动模式
target_motion_mode = 2;      % 1=静止目标, 2=匀速直线运动目标

% UAV kinematics and limits
v_max      = 15.0;           % [m/s]
v_min      = 0.0;            % [m/s]
a_max      = 8.0;            % [m/s^2] accel magnitude limit for speed changes
omega_max  = 160*pi/180;     % [rad/s] max yaw rate
pitch_max  = 60*pi/180;      % [rad]   pitch limit (nose up/down)
q_max      = 100*pi/180;     % [rad/s] max pitch rate

% Visual-servo (image-plane) PD (FOV holding)
kp_ex      = 0.30;           % yaw PD on ex (approx ~ azimuth) — primary FOV holding
kd_ex      = 0.08;           % yaw D on ex_dot

% PNG-like guidance on LOS angles
Ky         = 4.5;            % vertical plane navigation constant (~2-6)
Kz         = 4.5;            % horizontal plane navigation constant (~2-6)

% Blend PNG yaw/pitch tracking vs. FOV yaw holding (0..1)
w_nav_yaw  = 0.45;           % contribution of nav yaw alignment
w_fov_yaw  = 1.0 - w_nav_yaw;

w_nav_pitch= 0.70;           % pitch mostly follows nav (vertical guidance)

% Closing-speed shaping & LOS vertical motion damping
k_v        = 1.1;            % base closing-speed gain
ka_speed   = 2.0;            % extra velocity gain to keep v-axis motion small (acts via rate limiting)

% Vertical (independent climb) control for quadrotor
vz_max     = 3.0;            % [m/s] max climb/descent rate
az_vz_max  = 6.0;            % [m/s^2] max vertical acceleration (rate limit for vz)
k_vz       = 0.8;            % [1/s] vertical P gain on z error
search_vz  = 0.0;            % [m/s] vertical bias while searching (usually 0)

% FOV settings (3D frustum)
fov_deg    = 45;                      % full angle (aperture)
fov        = deg2rad(fov_deg);        % [rad]
R_fov_near = 5.0;                     % [m] near plane distance
R_fov_far  = 50.0;                    % [m] far plane distance
n_slices   = 4;                       % number of grid slices between near & far
slice_grid = 9;                       % grid resolution per slice (NxN)

% Impact (collision) radius - 增大撞击半径以提高成功率
r_hit      = 2.5;            % [m] 从1.5增加到2.5

% Search behavior when target is outside FOV
search_yaw_rate   = 25*pi/180;   % [rad/s] constant yaw scan
search_pitch_amp  = 8*pi/180;    % [rad]   pitch oscillation amplitude
search_pitch_omega= 0.7*2*pi;    % [rad/s] pitch oscillation speed
search_speed      = 2.8;         % [m/s]

% Quadrotor render geometry (body frame, x_fwd, y_left, z_up)
arm_len   = 1.2;             % distance from center to ring center (in XY plane)
ring_R    = 0.35;            % ring radius
n_ringpts = 40;              % circle discretization

% Target motion (3D) - 根据选择的目标模式设置
pt0 = [25; 10; 5];            % [m]
if target_motion_mode == 1
    % 静止目标
    vt = [0; 0; 0];           % [m/s]
else
    % 匀速直线运动目标
    vt = [0.5; 0.0; 0.5];     % [m/s]
end

% Initial UAV state: [x;y;z;yaw;pitch;v]
x0 = [-20; -10;  4; deg2rad(30); deg2rad(5); 0.0]; 
% Measurement noise on azimuth/elevation
add_noise        = true;
az_std_deg       = 0.6;    % [deg]
el_std_deg       = 0.6;    % [deg]

% Simple Delayed-KF (DKF-lite) on [az, el]
meas_latency_s   = 0.08;                 % emulate image pipeline delay [s]
M_DELAY          = max(0, round(meas_latency_s/DT));
Q_azel           = diag([2e-4, 1e-3, 2e-4, 1e-3]); % process q for [az,az_dot,el,el_dot]
R_azel           = diag((deg2rad([az_std_deg, el_std_deg]).^2)); % meas noise for [az,el]

%% Pre-alloc
x  = zeros(6, N+1); x(:,1) = x0;
pt = zeros(3, N+1); pt(:,1) = pt0;

az_hist  = nan(1, N+1); el_hist  = nan(1, N+1);
range_hist = nan(1, N+1); see_hist = false(1, N+1);

% History for PNG (previous LOS angles in world frame)
qy_prev = NaN; qz_prev = NaN;    % LOS elevation & custom azimuth
sy_prev = NaN; sz_prev = NaN;    % velocity angles

% DKF-lite state [az; az_dot; el; el_dot]
KF.x = [0;0;0;0]; KF.P = eye(4)*1e-2;
A = [1 DT 0  0; 0 1 0  0; 0 0 1 DT; 0 0 0 1];
H = [1 0 0 0; 0 0 1 0];
KF.Q = Q_azel; KF.R = R_azel;

% Measurement delay buffer (FIFO of raw [az; el])
az_buf = zeros(1, M_DELAY+1);
el_buf = zeros(1, M_DELAY+1);

%% --- Camera (on chaser UAV) parameters [ADD] ---
IMG_W = 640;                 % [px]
IMG_H = 480;                 % [px]
f_cam = (IMG_W/2)/tan(fov/2);% focal length from FOV (pinhole)
cx = IMG_W/2; cy = IMG_H/2;  % principal point
% bbox size heuristic vs. range (apparent size ~ 1/R)
r_tgt = 1.6;                 % (reuse sphere radius as "target size" proxy)
bbox_scale = 2.0 * f_cam * r_tgt;

%% Figure & 3D axes
fig = figure('Color','w','Name','IBVS+PNG+DKF');
ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
axis(ax,[-60 100 -60 80  -5 60]); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
view(45,22); camproj('perspective');

% Ground plane (for depth cue)
[xg,yg] = meshgrid(linspace(-60,100,10), linspace(-60,80,10)); zg = zeros(size(xg));
surf(ax, xg, yg, zg, 'FaceColor',[0.96 0.96 0.96], 'EdgeColor',[0.85 0.85 0.85], 'FaceAlpha',0.4);

% Trails
hTrailUAV = plot3(ax, nan, nan, nan, '-', 'LineWidth', 1.5);
hTrailTgt = plot3(ax, nan, nan, nan, '-', 'LineWidth', 1.5);

% Target marker (sphere)
[tgx, tgy, tgz] = sphere(16);
r_tgt = 0.6; 
hTgt = surf(ax, r_tgt*tgx+pt(1,1), r_tgt*tgy+pt(2,1), r_tgt*tgz+pt(3,1), ...
    'EdgeColor','none', 'FaceColor', [1 0.2 0.2], 'FaceAlpha',0.9); % 使用更鲜艳的红色

% === Quadrotor wire model (body frame, x_fwd, y_left, z_up)
[arms_b, rings_b] = uav_quad_geom(arm_len, ring_R, n_ringpts);
hArms(1) = plot3(ax, arms_b{1}(:,1), arms_b{1}(:,2), arms_b{1}(:,3), '-', 'LineWidth', 3.0, 'Color',[0.1 0.2 0.7]);
hArms(2) = plot3(ax, arms_b{2}(:,1), arms_b{2}(:,2), arms_b{2}(:,3), '-', 'LineWidth', 3.0, 'Color',[0.1 0.2 0.7]);
for i=1:4
    hRings(i) = plot3(ax, rings_b{i}(:,1), rings_b{i}(:,2), rings_b{i}(:,3), '-', 'LineWidth', 2.0, 'Color',[0.0 0.0 0.0]);
end

% === Frustum FOV (body frame primitives)
[near_b, far_b, edge_idx] = fov_frustum_edges(R_fov_near, R_fov_far, fov/2);
hFOVEdges = gobjects(size(edge_idx,1),1);
for i=1:size(edge_idx,1)
    hFOVEdges(i) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 1.2, 'Color', [0.1 0.6 0.2]);
end
[sliceXb, sliceYb, sliceZb] = frustum_slices_body(R_fov_near, R_fov_far, fov/2, n_slices, slice_grid);
hFOVSlices = gobjects(n_slices,1);
for i=1:n_slices
    hFOVSlices(i) = surf(ax, sliceXb{i}, sliceYb{i}, sliceZb{i}, 'EdgeColor',[0.2 0.6 0.3], 'FaceColor',[0.6 0.9 0.6], 'FaceAlpha',0.15);
end

% HUD text (3D)
hTxt = text(ax, 0.02, 0.98, '', 'Units','normalized', 'HorizontalAlignment','left', ...
    'VerticalAlignment','top', 'FontName','monospaced', 'FontSize', 11);

light('Position',[1 1 1]); lighting gouraud;

%% --- Camera View Figure (chaser UAV) [ADD] ---
figCam = figure('Color','k','Name','Chaser Camera View');
axCam  = axes('Parent',figCam); hold(axCam,'on');
axis(axCam,[0 IMG_W 0 IMG_H]); axis(axCam,'ij'); axis(axCam,'image');
axCam.XColor = [0.8 0.8 0.8]; axCam.YColor = [0.8 0.8 0.8];
set(figCam,'Color','w');
set(axCam,'Color','w');

% FOV frame
rectangle('Position',[1 1 IMG_W-2 IMG_H-2],'EdgeColor',[0.3 0.6 1.0],'LineStyle','-','LineWidth',1.0,'Parent',axCam);
% Crosshair
hCrossH = line(axCam,[0 IMG_W],[cy cy],'LineStyle',':','LineWidth',0.8,'Color',[0.6 0.6 0.6]);
hCrossV = line(axCam,[cx cx],[0 IMG_H],'LineStyle',':','LineWidth',0.8,'Color',[0.6 0.6 0.6]);
% Target dot + bbox (init off-screen)
hDot  = plot(axCam,-10,-10,'o','MarkerSize',6,'MarkerFaceColor',[1 0.3 0.3],'MarkerEdgeColor','none');
hBBox = rectangle('Position',[ -10 -10 1 1 ],'EdgeColor',[1 0.4 0.2],'LineWidth',1.2,'Curvature',[0.08 0.08],'Parent',axCam);
% HUD text
hHUD  = text(axCam,8,16,'','Color',[0.9 0.9 0.9],'FontName','monospaced','FontSize',10,'Interpreter','tex');

%% Main loop
impact = false;
% Initialize vertical speed state (separate from forward speed)
vz_prev = 0.0;
for k = 1:N
    t = (k-1)*DT;

    % Target update
    pt(:,k+1) = pt(:,k) + vt*DT;

    % Relative geometry (world)
    r_w = pt(:,k) - x(1:3,k); R = norm(r_w);
    nt = r_w / max(1e-9, R); % LOS unit in world

    % Body-from-world rotation (yaw then pitch, x_fwd, y_left, z_up)
    R_wb = Rz(x(4,k)) * Ry(x(5,k));  % world_from_body
    R_bw = R_wb.';                   % body_from_world

    % Bearing in body frame (for FOV & image-plane)
    r_b = R_bw * r_w; rhat_b = r_b / max(1e-9, norm(r_b));
    az_raw  = atan2(r_b(2), r_b(1));                          % azimuth
    el_raw  = atan2(r_b(3), hypot(r_b(1), r_b(2)));           % elevation

    if add_noise
        az_raw = az_raw + deg2rad(az_std_deg)*randn;
        el_raw = el_raw + deg2rad(el_std_deg)*randn;
    end

    % ---- DKF-lite with measurement latency ----
    az_buf = [az_buf(2:end), az_raw];
    el_buf = [el_buf(2:end), el_raw];
    meas_az = az_buf(1); meas_el = el_buf(1); % delayed measurement

    % Predict
    KF.x = A*KF.x; KF.P = A*KF.P*A' + KF.Q;
    % Update with delayed meas
    z = [meas_az; meas_el];
    yk = z - H*KF.x; S = H*KF.P*H' + KF.R; Kk = KF.P*H'/S;
    KF.x = KF.x + Kk*yk; KF.P = (eye(4)-Kk*H)*KF.P;

    az = KF.x(1); el = KF.x(3);  % filtered angles used for control

    % FOV check: angle to +x_b axis & within near/far
    cosang = rhat_b(1); % dot([1;0;0], rhat_b)
    inFOV  = (acos(max(-1,min(1,cosang))) <= fov/2) && (R <= R_fov_far) && (R >= R_fov_near) && (r_b(1) > 0);

    %% --- Update Camera View [ADD] ---
    % Project (filtered) angles to pixel: u = f*tan(az) + cx, v = f*tan(el) + cy
    u_px = f_cam * tan(az) + cx;
    v_px = f_cam * tan(el) + cy;

    % Heuristic bbox size vs. range (apparent size ~ 1/R)
    px_span = max(4, bbox_scale / max(R,1e-3));  % clamp min size
    bbox_w = px_span; bbox_h = px_span;

    if inFOV && isfinite(u_px) && isfinite(v_px) && u_px>=0 && u_px<=IMG_W && v_px>=0 && v_px<=IMG_H
        set(hDot,'XData',u_px,'YData',v_px,'Visible','on');
        set(hBBox,'Position',[u_px - bbox_w/2, v_px - bbox_h/2, bbox_w, bbox_h],'Visible','on');
        hud_str = sprintf('t = %5.2f s  |  R = %6.2f m  |  az = %6.1f^\\circ  el = %6.1f^\\circ  |  inFOV = YES', ...
                           t, R, rad2deg(az), rad2deg(el));
    else
        % Hide / move off-screen when out of FOV
        set(hDot,'XData',-10,'YData',-10,'Visible','on');
        set(hBBox,'Position',[-10 -10 1 1],'Visible','on');
        hud_str = sprintf('t = %5.2f s  |  R = %6.2f m  |  az = %6.1f^\\circ  el = %6.1f^\\circ  |  inFOV = NO', ...
                           t, R, rad2deg(az), rad2deg(el));
    end
    set(hHUD,'String',hud_str);

    % === PNG-like desired velocity direction (world)
    % LOS angles in world
    qy = atan2(nt(3), hypot(nt(1), nt(2)));    % elevation of LOS
    qz = atan2(nt(1), nt(2));                  % custom azimuth (x over y)

    % Velocity direction vector & angles (world)
    nv_now = R_wb * [1;0;0]; nv = nv_now / norm(nv_now);
    sy = atan2(nv(3), hypot(nv(1), nv(2)));    % elevation of velocity
    sz = atan2(nv(1), nv(2));                  % custom azimuth of velocity

    if isnan(qy_prev), qy_prev = qy; end
    if isnan(qz_prev), qz_prev = qz; end
    if isnan(sy_prev), sy_prev = sy; end
    if isnan(sz_prev), sz_prev = sz; end

    % PNG integration (discrete form)
    syd = Ky*(qy - qy_prev) + sy_prev;
    szd = Kz*(qz - qz_prev) + sz_prev;

    % Desired velocity direction unit (world)
    nvd = [cos(syd)*sin(szd);  % x
           cos(syd)*cos(szd);  % y
           sin(syd)];          % z
    nvd = nvd / max(1e-9, norm(nvd));

    % Desired closing speed with alignment scaling & vertical damping
    align_scale = max(0.15, cos(abs(az))*cos(abs(el))); % IBVS alignment proxy
    v_des = min(v_max, k_v*R) * align_scale;

    % Additional damping of vertical image motion via tighter rate-limit
    a_eff = max(0.5, min(a_max, ka_speed));
    v_cmd = clamp(v_des, v_min, v_max);

    % === Attitude rate commands (blend nav and FOV holding)
    % Desired yaw/pitch from nvd
    yaw_des   = atan2(nvd(2), nvd(1));
    pitch_des = atan2(nvd(3), hypot(nvd(1), nvd(2)));
    yaw_err   = wrapToPi(yaw_des - x(4,k));
    pitch_err = clamp(pitch_des - x(5,k), -pi/2, pi/2);

    % FOV yaw PD (ex ≈ az, ex_dot ≈ KF.x(2))
    ex = az; ex_dot = KF.x(2);
    yaw_rate_nav = 1.8 * yaw_err;                          % proportional yaw towards desired heading
    yaw_rate_fov = kp_ex*ex + kd_ex*ex_dot;                % keep target centered horizontally

    yaw_rate_cmd   = w_nav_yaw * yaw_rate_nav + w_fov_yaw * yaw_rate_fov;
    yaw_rate_cmd   = sat(yaw_rate_cmd, -omega_max, omega_max);

    pitch_rate_cmd = w_nav_pitch * (1.6 * pitch_err);      % proportional pitch towards desired vertical dir
    pitch_rate_cmd = sat(pitch_rate_cmd, -q_max, q_max);

    % === Commanded speed with rate limit (acceleration constraint)
    v_next = rate_limit(x(6,k), v_cmd, a_eff, DT);

    % === Vertical climb control (independent z DOF)
    z_err = pt(3,k) - x(3,k);
    if inFOV
        vz_cmd = k_vz * z_err;
    else
        vz_cmd = search_vz;
    end
    vz_cmd  = sat(vz_cmd, -vz_max, vz_max);
    vz_next = rate_limit(vz_prev, vz_cmd, az_vz_max, DT);

    % === Integrate ===
    yaw_next   = wrapToPi(x(4,k) + yaw_rate_cmd*DT);
    pitch_next = clamp(x(5,k) + pitch_rate_cmd*DT, -pitch_max, pitch_max);

    % Forward direction in world (from updated attitude)
    fwd_w = Rz(yaw_next) * Ry(pitch_next) * [1;0;0];

    % Integrate position
    p_next = x(1:3,k) + (fwd_w * v_next + [0;0;vz_next]) * DT;

    % Save state
    x(:,k+1) = [p_next; yaw_next; pitch_next; v_next];

    % Save vertical speed for next step
    vz_prev = vz_next;

    % Logs
    az_hist(k)    = az;  el_hist(k) = el; range_hist(k) = R; see_hist(k) = inFOV;

    % Update prev for PNG
    qy_prev = qy; qz_prev = qz; sy_prev = sy; sz_prev = sz;

    % Impact check
    if R <= r_hit
        impact = true; %#ok<NASGU>
        draw_frame(ax, hArms, hRings, hFOVEdges, hFOVSlices, hTgt, hTrailUAV, hTrailTgt, hTxt, ...
            x(:,k+1), pt(:,k+1), arms_b, rings_b, near_b, far_b, edge_idx, sliceXb, sliceYb, sliceZb, t+DT, R, true);
        title(ax, sprintf('IMPACT at t=%.2f s | range=%.2f m', t, R), 'FontWeight','bold');
        break;
    end

    % Animate (3D)
    draw_frame(ax, hArms, hRings, hFOVEdges, hFOVSlices, hTgt, hTrailUAV, hTrailTgt, hTxt, ...
        x(:,k+1), pt(:,k+1), arms_b, rings_b, near_b, far_b, edge_idx, sliceXb, sliceYb, sliceZb, t+DT, R, false, x(:,1:k+1), pt(:,1:k+1));

    if REALTIME, pause(DT*0.85); else, drawnow limitrate; end
end

if ~impact
    title(ax, sprintf('Timeout. Min range = %.2f m', min(range_hist(~isnan(range_hist)))), 'FontWeight','bold');
    plot3(x(1,1:k),x(2,1:k),x(3,1:k),'b-','LineWidth',1.5);
    plot3(pt(1,1:k),pt(2,1:k),pt(3,1:k),'r-','LineWidth',1.5);
end

%% Logs - 确保第三幅图显示
figure('Color','w','Name','Logs (3D, IBVS+PNG+Frustum)');
time_axis = (0:k-1)*DT;
subplot(3,1,1); 
plot(time_axis, rad2deg(az_hist(1:k)), 'LineWidth',1.2); 
grid on; ylabel('Az [deg]'); title('Azimuth Angle');

subplot(3,1,2); 
plot(time_axis, rad2deg(el_hist(1:k)), 'LineWidth',1.2); 
grid on; ylabel('El [deg]'); title('Elevation Angle');

subplot(3,1,3); 
plot(time_axis, range_hist(1:k), 'LineWidth',1.2); 
grid on; ylabel('Range [m]'); xlabel('t [s]'); title('Range to Target');

% 添加目标模式信息到标题
if target_motion_mode == 1
    mode_str = 'Stationary Target';
else
    mode_str = 'Moving Target';
end
sgtitle(sprintf('UAV Visual Servoing Performance - %s', mode_str), 'FontSize', 14, 'FontWeight', 'bold');

end % main

%% ===== Helpers =====
function [arms_b, rings_b] = uav_quad_geom(arm_len, ring_R, n_ringpts)
% Returns cell arrays of arms (2 cells: each Nx3) and 4 ring polylines (each Nx3)
% Arms lie in XY plane, crossing at 45 degrees to x-axis ("X" shape)
% Ring centers at arm ends; rings lie in planes roughly parallel to XY.

% Arm directions (unit) in XY
u1 = [cosd(45);  sind(45);  0];
u2 = [cosd(135); sind(135); 0];

c  = [0;0;0];
P1a = c - arm_len*u1; P1b = c + arm_len*u1; % arm 1 endpoints
P2a = c - arm_len*u2; P2b = c + arm_len*u2; % arm 2 endpoints

arms_b = {
    [linspace(P1a(1),P1b(1),2)', linspace(P1a(2),P1b(2),2)', linspace(P1a(3),P1b(3),2)'];
    [linspace(P2a(1),P2b(1),2)', linspace(P2a(2),P2b(2),2)', linspace(P2a(3),P2b(3),2)'];
};

% Ring centers (4 tips)
centers = [P1b, P2b, P1a, P2a];
angles  = linspace(0, 2*pi, n_ringpts)';
ring_xy = [cos(angles), sin(angles), zeros(size(angles))];

rings_b = cell(1,4);
for i=1:4
    Rz45 = eye(3); % keep rings parallel to XY plane for clean look
    pts = (Rz45 * (ring_R * ring_xy)')' + centers(:,i)';
    rings_b{i} = pts;
end
end

function [near_b, far_b, edge_idx] = fov_frustum_edges(Lnear, Lfar, half_angle)
% Square frustum in body frame: apex would be at origin; we keep near/far squares
wN = Lnear * tan(half_angle);
wF = Lfar  * tan(half_angle);
near_b = [ Lnear,  wN,  wN;
           Lnear, -wN,  wN;
           Lnear, -wN, -wN;
           Lnear,  wN, -wN];
far_b  = [ Lfar,   wF,  wF;
           Lfar,  -wF,  wF;
           Lfar,  -wF, -wF;
           Lfar,   wF, -wF];
% edges: near square, far square, and 4 connecting edges
edge_idx = [ 1 2; 2 3; 3 4; 4 1; ...      % near
             5 6; 6 7; 7 8; 8 5; ...      % far (indices 5..8 will map to far)
             1 5; 2 6; 3 7; 4 8];         % connections
% NOTE: when using edge_idx, you'll map indices 1..4->near, 5..8->far
end

function [sliceXb, sliceYb, sliceZb] = frustum_slices_body(Lnear, Lfar, half_angle, n_slices, gridN)
% Precompute slice surfaces in body frame (constant-x planes with square bounds)
ss = linspace(Lnear, Lfar, n_slices);
sliceXb = cell(n_slices,1); sliceYb = cell(n_slices,1); sliceZb = cell(n_slices,1);
for i=1:n_slices
    s = ss(i);
    w = s * tan(half_angle);
    y = linspace(-w, w, gridN);
    z = linspace(-w, w, gridN);
    [YY, ZZ] = meshgrid(y, z);
    XX = s * ones(size(YY));
    sliceXb{i} = XX; sliceYb{i} = YY; sliceZb{i} = ZZ;
end
end

function draw_frame(ax, hArms, hRings, hFOVEdges, hFOVSlices, hTgt, hTrailUAV, hTrailTgt, hTxt, ...
        x, pt, arms_b, rings_b, near_b, far_b, edge_idx, sliceXb, sliceYb, sliceZb, t, R, impact, xhist, pthist)
if nargin < 23, xhist = []; pthist = []; end

% Transform helpers
R_wb = Rz(x(4)) * Ry(x(5));  % world_from_body

% Update quadrotor arms (2 lines)
for i=1:2
    Pw = transform_points(arms_b{i}, R_wb, x(1:3));
    set(hArms(i), 'XData', Pw(:,1), 'YData', Pw(:,2), 'ZData', Pw(:,3));
end
% Update rings (4 circles)
for i=1:4
    Pw = transform_points(rings_b{i}, R_wb, x(1:3));
    set(hRings(i), 'XData', Pw(:,1), 'YData', Pw(:,2), 'ZData', Pw(:,3));
end

% Transform frustum edge points
Pts_b = [near_b; far_b];
Pts_w = transform_points(Pts_b, R_wb, x(1:3));

% Update edge lines
for i=1:size(edge_idx,1)
    a = edge_idx(i,1); b = edge_idx(i,2);
    Pa = Pts_w(a,:); Pb = Pts_w(b,:);
    set(hFOVEdges(i), 'XData', [Pa(1) Pb(1)], 'YData', [Pa(2) Pb(2)], 'ZData', [Pa(3) Pb(3)]);
end

% Update grid slices (semi-transparent planes)
for i=1:numel(hFOVSlices)
    [Xw,Yw,Zw] = transform_surf(sliceXb{i}, sliceYb{i}, sliceZb{i}, R_wb, x(1:3));
    set(hFOVSlices(i), 'XData', Xw, 'YData', Yw, 'ZData', Zw);
end

% Update target sphere
set(hTgt, 'XData', get(hTgt,'XData')*0 + pt(1));
set(hTgt, 'YData', get(hTgt,'YData')*0 + pt(2));
set(hTgt, 'ZData', get(hTgt,'ZData')*0 + pt(3));

% Trails
if ~isempty(xhist)
    set(hTrailUAV, 'XData', xhist(1,:), 'YData', xhist(2,:), 'ZData', xhist(3,:), 'Color',[0.1 0.3 0.8]);
    set(hTrailTgt, 'XData', pthist(1,:), 'YData', pthist(2,:), 'ZData', pthist(3,:), 'Color',[0.8 0.2 0.2]);
end

% HUD (3D)
s = sprintf('t = %5.2f s | range = %6.2f m | v = %5.2f m/s | yaw = %6.1f deg | pitch = %6.1f deg', ...
             t, R, x(6), rad2deg(x(4)), rad2deg(x(5)));
set(hTxt, 'String', s);

% Title
if impact
    title(ax, 'UAV Visual Servoing (3D) — IMPACT', 'FontWeight','bold');
else
    title(ax, 'IBVS+PNG+DKF', 'FontWeight','bold');
end

drawnow limitrate;
end

function [Xw,Yw,Zw] = transform_surf(Xb,Yb,Zb, R_wb, p)
% Apply rotation+translation to a parametric surface
sz = size(Xb);
Pw = R_wb * [Xb(:)'; Yb(:)'; Zb(:)'] + p(:);
Xw = reshape(Pw(1,:), sz); Yw = reshape(Pw(2,:), sz); Zw = reshape(Pw(3,:), sz);
end

function Pw = transform_points(Pb, R_wb, p)
% Transform Nx3 points from body to world
if size(Pb,2) == 3
    Pw = (R_wb * Pb.').' + p(:).';
else
    Pw = (R_wb * Pb')' + p(:)';
end
end

function M = Rz(yaw)
c = cos(yaw); s = sin(yaw);
M = [ c -s  0;
      s  c  0;
      0  0  1];
end

function M = Ry(pitch)
c = cos(pitch); s = sin(pitch);
M = [ c  0  s;
      0  1  0;
     -s  0  c];
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function y = sat(x, lo, hi)
y = min(max(x, lo), hi);
end

function y = rate_limit(x_prev, x_cmd, a_max, dt)
max_delta = a_max * dt;
delta = x_cmd - x_prev;
if delta > max_delta
    y = x_prev + max_delta;
elseif delta < -max_delta
    y = x_prev - max_delta;
else
    y = x_cmd;
end
end

function ang = wrapToPi(ang)
ang = mod(ang + pi, 2*pi) - pi;
end