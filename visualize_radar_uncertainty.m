%% visualize_radar_uncertainty.m
%
% Animated 3D visualization of the RadarEmulator position-uncertainty
% ellipsoid as a target flies a straight horizontal trajectory.
%
% Physics shown:
%   - Along the line-of-sight (LOS) the error is small (≈ rmse_range)
%   - Cross-LOS error grows linearly with range (r × σ_angular)
%   - The ellipsoid therefore deforms from a sphere at short range to
%     an elongated "rugby ball" perpendicular to the LOS at long range
%
% Controls:
%   - Close the figure to stop the animation
%   - Adjust TRAJECTORY / RADAR / RMSE sections below to explore
%
% Requirements: RadarEmulator.m, Mapping Toolbox

clear; clc; close all;

%% ======================== RADAR SETUP ========================
ref_lla0 = [39.9; 32.9; 0];       % NED origin [lat,lon,alt] (deg,deg,m)
radar_offset = [0; 0; 0];         % Radar NED offset from ref_lla0 [m]

% Convert radar NED offset to LLA (needed by RadarEmulator constructor)
[ecef_rx, ecef_ry, ecef_rz] = ned2ecef( ...
    radar_offset(1), radar_offset(2), radar_offset(3), ...
    ref_lla0(1), ref_lla0(2), ref_lla0(3), referenceEllipsoid('wgs84'));
[r_lat, r_lon, r_alt] = ecef2geodetic( ...
    referenceEllipsoid('wgs84'), ecef_rx, ecef_ry, ecef_rz);
radar_lla = [r_lat; r_lon; r_alt];

% ---- Radar noise parameters (match run_eskf_simulation.m) ----
radar_params.rmse_range     = 3.0;
radar_params.rmse_azimuth   = deg2rad(1.3);   % ~1.3 deg
radar_params.rmse_elevation = deg2rad(3.3);   % ~3.3 deg
radar_params.rmse_doppler   = 0.5;
radar_params.radar_lla      = radar_lla;
radar_params.ref_lla0       = ref_lla0;
radar_params.dt             = 2.0;            % 0.5 Hz
radar = RadarEmulator(radar_params);

%% ======================== TARGET TRAJECTORY ========================
% Straight horizontal trajectory (vD = 0) passing near the radar
t_total   = 80;     % total simulation time [s]
dt_anim   = 0.3;    % time between animation frames [s]
t_vec     = 0 : dt_anim : t_total;
N         = numel(t_vec);

p0_tgt    = [-60; -120; -45];     % start position in NED [m] (alt = 45 m AGL)
v_tgt_ned = [3.5; 4.0; 0];        % constant horizontal velocity [vN, vE, 0] m/s

traj = p0_tgt + v_tgt_ned .* t_vec;   % 3×N

%% ======================== PRECOMPUTE ELLIPSOIDS ========================
sigma_scale = 3;        % draw 3-sigma ellipsoid
n_sph       = 28;       % sphere tessellation (higher = smoother)

[xs, ys, zs] = sphere(n_sph);
sphere_pts = [xs(:)'; ys(:)'; zs(:)'];     % 3×(n+1)²

% Storage
ell_X  = cell(N,1);   % surface X in display coords (North)
ell_Y  = cell(N,1);   % surface Y in display coords (East)
ell_Z  = cell(N,1);   % surface Z in display coords (Altitude, +up)

r_arr     = zeros(N,1);
theta_arr = zeros(N,1);
phi_arr   = zeros(N,1);
semiax    = zeros(N,3);   % 3-sigma semi-axes lengths from eigenvalues
sigma_ned = zeros(N,3);   % 1-sigma std in North, East, Down [m]

for i = 1:N
    p   = traj(:,i) - radar_offset;          % target relative to radar NED
    xn  = p(1);  xe = p(2);  xd = p(3);
    r   = max(norm(p), 0.1);
    th  = atan2(xe, xn);
    ph  = atan2(-xd, sqrt(xn^2 + xe^2));

    r_arr(i)     = r;
    theta_arr(i) = th;
    phi_arr(i)   = ph;

    [R_pos, ~] = radar.getCartesianCovariance(r, th, ph);

    % Eigen-decompose for principal axes
    [V, D] = eig(R_pos);
    eig_vals    = max(real(diag(D)), 0);   % guard tiny negatives
    semiax(i,:) = sigma_scale * sqrt(eig_vals)';   % semi-axis lengths

    % 1-sigma std in each NED Cartesian direction (sqrt of diagonal of R_pos)
    sigma_ned(i,:) = sqrt(max(diag(R_pos), 0))';

    % Map unit sphere -> ellipsoid surface in NED, then shift to target
    E_pts = sigma_scale * V * diag(sqrt(eig_vals)) * sphere_pts;

    sh = n_sph + 1;
    Xn = reshape(E_pts(1,:) + traj(1,i), sh, sh);
    Xe = reshape(E_pts(2,:) + traj(2,i), sh, sh);
    Xd = reshape(E_pts(3,:) + traj(3,i), sh, sh);

    % Display convention: flip Down → Altitude (+up)
    ell_X{i} = Xn;
    ell_Y{i} = Xe;
    ell_Z{i} = -Xd;   % altitude = -Down
end

% Display trajectory & radar in altitude-up coords
traj_N = traj(1,:);
traj_E = traj(2,:);
traj_A = -traj(3,:);   % altitude

radar_N = radar_offset(1);
radar_E = radar_offset(2);
radar_A = -radar_offset(3);

%% ======================== FIGURE & LAYOUT ========================
fig = figure( ...
    'Name',     'Radar 3D Uncertainty Ellipsoid Visualizer', ...
    'Color',    [0.10 0.10 0.13], ...
    'Position', [30 30 1550 860]);

tlo = tiledlayout(fig, 3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

% 3D scene spans left 3 columns all 3 rows
ax = nexttile(tlo, 1, [3 3]);
ax.Color     = [0.08 0.08 0.11];
ax.XColor    = [0.65 0.65 0.70];
ax.YColor    = [0.65 0.65 0.70];
ax.ZColor    = [0.65 0.65 0.70];
ax.GridColor = [0.28 0.28 0.33];
ax.GridAlpha = 0.45;
ax.FontSize  = 10;
hold(ax,'on'); grid(ax,'on'); box(ax,'on');

xlabel(ax, 'North [m]', 'Color', [0.82 0.82 0.85], 'FontSize', 11);
ylabel(ax, 'East  [m]', 'Color', [0.82 0.82 0.85], 'FontSize', 11);
zlabel(ax, 'Altitude [m]', 'Color', [0.82 0.82 0.85], 'FontSize', 11);
title(ax, {'Radar 3\sigma Position-Uncertainty Ellipsoid', ...
       '\rm\color[rgb]{0.6,0.7,0.9}Small along LOS  ·  Grows cross-range with distance'}, ...
    'Color', [1 1 1], 'FontSize', 13, 'FontWeight', 'bold');

%% ======================== STATIC ELEMENTS ========================

% ---- NED sigma subplots (right column of tiledlayout) ----
ned_clr = {[0.30 0.92 0.45], [0.30 0.65 1.00], [1.00 0.60 0.25]};
ned_lbl = {'  sigma_N [m]', '  sigma_E [m]', '  sigma_D [m]'};
ax_ned  = gobjects(3,1);
h_cur   = gobjects(3,1);

for d = 1:3
    ax_ned(d) = nexttile(tlo, d * 4, [1 1]);
    ax_ned(d).Color     = [0.08 0.08 0.11];
    ax_ned(d).XColor    = [0.55 0.55 0.60];
    ax_ned(d).YColor    = [0.55 0.55 0.60];
    ax_ned(d).GridColor = [0.25 0.25 0.30];
    ax_ned(d).GridAlpha = 0.50;
    ax_ned(d).FontSize  = 9;
    hold(ax_ned(d),'on'); grid(ax_ned(d),'on'); box(ax_ned(d),'on');

    % Shaded 3-sigma band
    patch(ax_ned(d), ...
        [t_vec, fliplr(t_vec)], ...
        [sigma_scale*sigma_ned(:,d)', fliplr(-sigma_scale*sigma_ned(:,d)')], ...
        ned_clr{d}, 'FaceAlpha', 0.10, 'EdgeColor', 'none');

    % +/- 3-sigma solid boundary
    plot(ax_ned(d), t_vec,  sigma_scale*sigma_ned(:,d)', ...
        '-', 'Color', [ned_clr{d}, 0.85], 'LineWidth', 1.4);
    plot(ax_ned(d), t_vec, -sigma_scale*sigma_ned(:,d)', ...
        '-', 'Color', [ned_clr{d}, 0.85], 'LineWidth', 1.4);

    % +/- 1-sigma dashed
    plot(ax_ned(d), t_vec,  sigma_ned(:,d)', ...
        '--', 'Color', [ned_clr{d}, 0.45], 'LineWidth', 0.9);
    plot(ax_ned(d), t_vec, -sigma_ned(:,d)', ...
        '--', 'Color', [ned_clr{d}, 0.45], 'LineWidth', 0.9);

    % Zero reference
    yline(ax_ned(d), 0, ':', 'Color', [0.45 0.45 0.50]);

    ylabel(ax_ned(d), ned_lbl{d}, 'Color', ned_clr{d}, 'FontSize', 9);
    xlim(ax_ned(d), [t_vec(1), t_vec(end)]);

    % Animated cursor (moving vertical line)
    h_cur(d) = xline(ax_ned(d), t_vec(1), ...
        'Color', [1 1 1 0.75], 'LineWidth', 1.6);
end
xlabel(ax_ned(3), 'Time [s]', 'Color', [0.70 0.70 0.75], 'FontSize', 9);
title(ax_ned(1), '3\sigma (solid)  &  1\sigma (dashed) NED Error Bounds', ...
    'Color', [0.90 0.92 1.00], 'FontSize', 9, 'FontWeight', 'bold');

% Make sure 3D axes is active and ready
axes(ax);
hold(ax, 'on');

% ---- Ghost trajectory (full path, faint) ----
plot3(ax, traj_N, traj_E, traj_A, '--', ...
    'Color', [0.40 0.60 1.00 0.35], 'LineWidth', 1.2);

% ---- Ghost ellipsoids at N_snap evenly-spaced snapshots ----
N_snap   = 6;
snap_idx = round(linspace(1, N, N_snap+2));
snap_idx = snap_idx(2:end-1);   % skip first/last (covered by animation)

for idx = snap_idx
    surf(ax, ell_X{idx}, ell_Y{idx}, ell_Z{idx}, ...
        'FaceAlpha', 0.06, ...
        'EdgeColor', [0.45 0.60 0.90], ...
        'EdgeAlpha', 0.07, ...
        'FaceColor', [0.40 0.58 1.00]);
end

% ---- Radar station ----
scatter3(ax, radar_N, radar_E, radar_A, 280, [1.00 0.38 0.12], ...
    'filled', 'Marker', '^', 'LineWidth', 1);
text(ax, radar_N+2, radar_E+2, radar_A+4, '  RADAR', ...
    'Color', [1.00 0.58 0.30], 'FontSize', 11, 'FontWeight', 'bold');

% ---- Ground plane grid ring around radar ----
ang = linspace(0, 2*pi, 120);
for R_ring = [50 100 150 200]
    plot3(ax, R_ring*cos(ang), R_ring*sin(ang), zeros(1,120), ...
        ':', 'Color', [0.30 0.30 0.35 0.5], 'LineWidth', 0.7);
    text(ax, R_ring, 0, 0, sprintf('%dm', R_ring), ...
        'Color', [0.40 0.40 0.45], 'FontSize', 8);
end

%% ======================== ANIMATED ELEMENTS ========================

% Active ellipsoid
h_ell = surf(ax, ell_X{1}, ell_Y{1}, ell_Z{1}, ...
    'FaceAlpha', 0.40, ...
    'EdgeColor', [0.35 0.55 1.00], ...
    'EdgeAlpha', 0.25, ...
    'FaceColor', [0.25 0.55 1.00]);
lighting(ax, 'gouraud');
camlight(ax, 'headlight');

% LOS line from radar to target
h_los = plot3(ax, [radar_N traj_N(1)], [radar_E traj_E(1)], [radar_A traj_A(1)], ...
    '-', 'Color', [1.00 0.88 0.20 0.65], 'LineWidth', 1.2);

% Vertical drop line from target to ground (altitude indicator)
h_drop = plot3(ax, [traj_N(1) traj_N(1)], [traj_E(1) traj_E(1)], [traj_A(1) 0], ...
    ':', 'Color', [0.35 1.00 0.55 0.40], 'LineWidth', 1.0);

% Moving target marker
h_tgt = scatter3(ax, traj_N(1), traj_E(1), traj_A(1), ...
    160, [0.20 1.00 0.50], 'filled', 'Marker', 'o');

% Trajectory trace (builds up as target moves)
h_trace = plot3(ax, traj_N(1), traj_E(1), traj_A(1), ...
    '-', 'Color', [0.25 0.90 0.50], 'LineWidth', 2.0);

% HUD info text
h_info = text(ax, 0.018, 0.97, '', ...
    'Units', 'normalized', ...
    'Color', [0.92 0.92 0.95], ...
    'FontSize', 10, ...
    'FontName', 'Courier New', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', [0.12 0.12 0.16 0.80], ...
    'Margin', 5, ...
    'Interpreter', 'none');

% LOS-axis indicator text (updated live)
h_axlbl = text(ax, 0.78, 0.97, '', ...
    'Units', 'normalized', ...
    'Color', [0.80 0.90 1.00], ...
    'FontSize', 10, ...
    'FontName', 'Courier New', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', [0.12 0.12 0.16 0.80], ...
    'Margin', 5, ...
    'Interpreter', 'none');

%% ======================== CAMERA SETUP ========================
view(ax, 38, 26);
camzoom(ax, 0.82);

% Nice axis limits
pad = 30;
axis(ax, [ min(traj_N)-pad   max(traj_N)+pad ...
           min(traj_E)-pad   max(traj_E)+pad ...
           -5               max(traj_A)+pad  ]);
drawnow;

%% ======================== ANIMATION LOOP ========================
fprintf('Running animation and recording MP4...\n');

% Initialize VideoWriter
v = VideoWriter('radar_uncertainty_video.mp4', 'MPEG-4');
v.FrameRate = 25;  % roughly match simulation speed
v.Quality = 100;
open(v);

trace_N = traj_N(1);
trace_E = traj_E(1);
trace_A = traj_A(1);

for i = 1:N
    if ~isvalid(fig), break; end

    % -- Ellipsoid --
    set(h_ell, 'XData', ell_X{i}, 'YData', ell_Y{i}, 'ZData', ell_Z{i});

    % -- Target marker --
    set(h_tgt, 'XData', traj_N(i), 'YData', traj_E(i), 'ZData', traj_A(i));

    % -- LOS line --
    set(h_los, 'XData', [radar_N traj_N(i)], ...
               'YData', [radar_E traj_E(i)], ...
               'ZData', [radar_A traj_A(i)]);

    % -- Drop line (altitude indicator) --
    set(h_drop, 'XData', [traj_N(i) traj_N(i)], ...
                'YData', [traj_E(i) traj_E(i)], ...
                'ZData', [traj_A(i) 0]);

    % -- Growing trace --
    trace_N(end+1) = traj_N(i); 
    trace_E(end+1) = traj_E(i); 
    trace_A(end+1) = traj_A(i); 
    set(h_trace, 'XData', trace_N, 'YData', trace_E, 'ZData', trace_A);

    % -- HUD text --
    info_str = sprintf( ...
        ['t      = %5.1f s\n' ...
         'Range  = %6.1f m\n' ...
         'Az     = %+7.2f deg\n' ...
         'El     = %+7.2f deg\n' ...
         'Alt    = %6.1f m'], ...
        t_vec(i), r_arr(i), ...
        rad2deg(theta_arr(i)), ...
        rad2deg(phi_arr(i)), ...
        traj_A(i));
    set(h_info, 'String', info_str);

    % Sorted semi-axes
    sa = sort(semiax(i,:));
    axlbl_str = sprintf( ...
        ['3-sigma semi-axes:\n' ...
         '  min = %5.1f m  (LOS)\n' ...
         '  mid = %5.1f m\n' ...
         '  max = %5.1f m  (cross)\n' ...
         '-------------------\n' ...
         '1-sigma NED:\n' ...
         '  sN = %5.2f m\n' ...
         '  sE = %5.2f m\n' ...
         '  sD = %5.2f m'], ...
        sa(1), sa(2), sa(3), ...
        sigma_ned(i,1), sigma_ned(i,2), sigma_ned(i,3));
    set(h_axlbl, 'String', axlbl_str);

    % -- NED cursor --
    for d = 1:3
        h_cur(d).Value = t_vec(i);
    end

    drawnow limitrate;
    
    % Capture frame and write to video
    frame = getframe(fig);
    writeVideo(v, frame);
end

% Close video file
close(v);
fprintf('Animation complete. Saved to ''radar_uncertainty_video.mp4''.\n');
