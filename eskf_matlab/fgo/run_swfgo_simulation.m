%% Sliding-Window Factor Graph Optimization (SW-FGO) for Target Tracking
% Online estimation using a fixed-size sliding window of variable nodes.
%
% This script solves the same target-tracking problem as run_fgo_simulation.m
% but replaces the batch (end-to-end) solve with a realistic online loop:
%
%   For each new time step k:
%       1.  Marginalise the oldest node out of the window (Schur complement)
%           when the window reaches capacity W.
%       2.  Append the new variable node, initialised by dead-reckoning.
%       3.  Attach dynamics / camera / radar factors.
%       4.  Run Gauss-Newton over the W-node window.
%       5.  Store estimates for all nodes currently in the window.
%
% State per variable node (6-dim):
%   x_k = [p_t(3);  v_t(3)]   target position and velocity
%
% Configuration parameter:
%   W  - Window size in FGO nodes (default 15 = 1.5 s at 10 Hz).
%        Increase for smoother estimates, decrease for lower latency/compute.

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));
rng(32)

%% ======================== SIMULATION PARAMETERS ========================
dt_truth  = 1/200;       % Truth propagation rate          [s]
dt_fgo    = 1/20;        % FGO node spacing                [s]
dt_cam    = 1/30;        % Camera measurement rate         [s]
dt_radar  = 1/0.5;       % Radar measurement rate          [s]
t_total   = 30;          % Total simulation time            [s]
t_delay   = 0/1000;     % Camera processing delay          [s]
D_nodes   = round(t_delay / dt_fgo);   % Delay in FGO nodes (= 1)

% -----------------------------------------------------------------------
%  SLIDING WINDOW SIZE  (key tuning parameter)
%    W =  5  ->  0.5 s window  (very local, fast, lowest accuracy)
%    W = 15  ->  1.5 s window  (default, good balance)
%    W = 30  ->  3.0 s window  (approaches batch quality)
% -----------------------------------------------------------------------
W = 15;

% Gauss-Newton iterations per step (low is fine — warm-started each step)
max_gn_iter = 10;

% Time vectors
t_truth = 0:dt_truth:t_total;  N_truth = length(t_truth);
t_fgo   = 0:dt_fgo  :t_total;  N_fgo   = length(t_fgo);
fgo_sample_ratio = round(dt_fgo / dt_truth);   % e.g. 20

%% ======================== PHYSICAL CONSTANTS ===========================
g  = 9.81;
e3 = [0; 0; 1];

%% ======================== CAMERA PARAMETERS ============================
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

%% ======================== SENSOR NOISE =================================
sigma_img = 0.05;
R_img     = sigma_img^2 * eye(2);

sigma_tgt_rw = 1.5;       % Target accel random walk [m/s^2]

radar_params.rmse_range     = 3.0;
radar_params.rmse_azimuth   = deg2rad(1.3);
radar_params.rmse_elevation = deg2rad(3.3);
radar_params.rmse_doppler   = 0.5;

R_radar = diag([radar_params.rmse_range^2 * ones(1,3), ...
                radar_params.rmse_doppler^2 * ones(1,3)]);

%% ======================== INITIAL CONDITIONS ===========================
p_int    = [0; 0; 0];
v_int    = [0; 1; -1];
yaw_init = 0;
q_true   = eul2quat([yaw_init, 0, 0], 'ZYX')';

p_tgt        = [160; 200; -50];
radius       = 70;
p_tgt_center = p_tgt - [radius; 0; 0];
v_tgt        = [0; 0; 0];

% Full truth state (21-dim, compatible with propagate_true_state)
p_r_true  = p_int - p_tgt;
v_r_true  = v_int - v_tgt;
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);
x_true_full = [q_true; p_r_true; v_r_true; pbar_true; ...
               zeros(3,1); zeros(3,1); zeros(3,1)];

idx_q         = 1:4;
idx_pr        = 5:7;
idx_vr        = 8:10;
idx_pbar_full = 11:12;
idx_bgyr      = 13:15;
idx_bacc      = 16:18;
idx_bmag      = 19:21;

%% ====================================================================
%  PHASE 1: GENERATE TRUTH TRAJECTORY  (at 200 Hz)
% =====================================================================
fprintf('Phase 1: Generating truth trajectory at %d Hz for %.0f s ...\n', ...
        round(1/dt_truth), t_total);

p_int_hist  = zeros(3, N_truth);
v_int_hist  = zeros(3, N_truth);
q_hist      = zeros(4, N_truth);
p_tgt_hist  = zeros(3, N_truth);
v_tgt_hist  = zeros(3, N_truth);
pbar_hist   = zeros(2, N_truth);

for k = 1:N_truth
    t = t_truth(k);
    p_int_hist(:,k) = p_int;
    v_int_hist(:,k) = v_int;
    q_hist(:,k)     = x_true_full(idx_q);
    p_tgt_hist(:,k) = p_tgt;
    v_tgt_hist(:,k) = v_tgt;
    pbar_hist(:,k)  = x_true_full(idx_pbar_full);
    if k < N_truth
        [x_true_full, p_int, v_int, p_tgt, v_tgt, ~, ~] = ...
            propagate_true_state(x_true_full, p_int, v_int, p_tgt, v_tgt, ...
                                 t, dt_truth, g, e3, R_b2c, ...
                                 idx_q, idx_pr, idx_vr, idx_pbar_full, ...
                                 idx_bgyr, idx_bacc, idx_bmag, p_tgt_center);
    end
end

%% ====================================================================
%  PHASE 2: SUB-SAMPLE TO FGO NODES & GENERATE MEASUREMENTS
% =====================================================================
fprintf('Phase 2: Sub-sampling to %d Hz and generating measurements ...\n', ...
        round(1/dt_fgo));

fgo_idx = 1:fgo_sample_ratio:N_truth;
fgo_idx = fgo_idx(1:N_fgo);

p_tgt_fgo = p_tgt_hist(:, fgo_idx);
v_tgt_fgo = v_tgt_hist(:, fgo_idx);
p_int_fgo = p_int_hist(:, fgo_idx);
v_int_fgo = v_int_hist(:, fgo_idx);
pbar_fgo  = pbar_hist(:, fgo_idx);

R_b2e_fgo = zeros(3, 3, N_fgo);
for k = 1:N_fgo
    R_b2e_fgo(:,:,k) = quat2rotm(q_hist(:, fgo_idx(k))');
end

% Camera measurements
z_cam     = zeros(2, N_fgo);
cam_valid = false(1, N_fgo);
for k = 1:N_fgo
    M     = R_b2c * R_b2e_fgo(:,:,k)';
    p_cam = M * (p_tgt_fgo(:,k) - p_int_fgo(:,k));
    if p_cam(3) > 2
        pbar_k       = [p_cam(1)/p_cam(3); p_cam(2)/p_cam(3)];
        z_cam(:,k)   = pbar_k + sigma_img * randn(2,1);
        cam_valid(k) = true;
    end
end
fprintf('  Camera delay = %.0f ms = %d FGO nodes\n', t_delay*1000, D_nodes);

% Radar measurements
radar_node_spacing = round(dt_radar / dt_fgo);
radar_nodes = 1:radar_node_spacing:N_fgo;
z_radar = zeros(6, length(radar_nodes));
for j = 1:length(radar_nodes)
    k = radar_nodes(j);
    z_radar(:,j) = [p_tgt_fgo(:,k); v_tgt_fgo(:,k)] + ...
                    chol(R_radar,'lower') * randn(6,1);
end
fprintf('  %d camera measurements, %d radar measurements\n', ...
        sum(cam_valid), length(radar_nodes));

%% ====================================================================
%  PHASE 3: ONLINE SLIDING-WINDOW OPTIMISATION
% =====================================================================
fprintf(['\nPhase 3: Online SW-FGO  (W = %d nodes = %.1f s) ...\n', ...
         '         D_nodes = %d  |  max_gn_iter = %d\n'], ...
        W, W*dt_fgo, D_nodes, max_gn_iter);

state_dim = 6;

% Process noise covariance (constant-velocity model)
dt     = dt_fgo;
sigma2 = sigma_tgt_rw^2;
Sigma_dyn = sigma2 * [dt^4/4*eye(3), dt^3/2*eye(3);
                      dt^3/2*eye(3), dt^2*eye(3)] + 1e-10*eye(6);
% Sigma_dyn = 1e-10*eye(6);
% Sigma_dyn(4:6,4:6) = eye(3) * sigma2*dt;

% Prior for node 1
x0_true    = [p_tgt_fgo(:,1); v_tgt_fgo(:,1)];
init_errors = [3; -2; 1.5; 0.3; -0.2; 0.1];
x0_prior   = x0_true + init_errors;
P_prior    = diag([25^2, 25^2, 25^2, 1^2, 1^2, 1^2]);

% Result storage
x_est      = zeros(state_dim, N_fgo);
P_diag     = zeros(state_dim, N_fgo);
step_costs  = zeros(N_fgo, 1);   % final GN cost at each time step

% ---- Initialise solver with first node ----
% sw = SlidingWindowFGO(W, state_dim);
sw = SimpleSlidingWindowFGO(W, state_dim);

sw.initFirstNode(1, x0_prior, P_prior);

% Add radar at node 1 if applicable
j_rad1 = find(radar_nodes == 1, 1);
if ~isempty(j_rad1)
    sw.addRadarFactor(1, z_radar(:,j_rad1), R_radar);
end

ch = sw.optimizeWindow(max_gn_iter, 1e-6);
step_costs(1) = ch(end);

x_est(:,1) = sw.getNewestState();
P_diag(:,1) = diag(sw.getNewestMarginal());

% ---- Main online loop ----
for k = 2:N_fgo

    % 1. Marginalise oldest node if window is at capacity
    if sw.n_active == sw.W
        sw.marginalizeOldestNode();
    end

    % 2. Dead-reckoning initialisation for new node
    x_prev     = sw.getNewestState();
    x_new_init = [x_prev(1:3) + x_prev(4:6)*dt; x_prev(4:6)];
    sw.addNewNode(k, x_new_init);

    % 3. Dynamics factor  (k-1) -> k
    sw.addDynamicsFactor(k-1, k, dt, Sigma_dyn);

    % 4. Camera factor  (delayed, attached to capture node)
    capture_k = k - D_nodes;
    if capture_k >= 1 && cam_valid(capture_k) && sw.isInWindow(capture_k)
        sw.addCameraFactor(capture_k, z_cam(:,capture_k), R_img, ...
                           R_b2c, R_b2e_fgo(:,:,capture_k), ...
                           p_int_fgo(:,capture_k));
    end

    % 5. Radar factor for node k
    j_rad = find(radar_nodes == k, 1);
    if ~isempty(j_rad)
        sw.addRadarFactor(k, z_radar(:,j_rad), R_radar);
    end

    % 6. Gauss-Newton over current window
    ch = sw.optimizeWindow(max_gn_iter, 1e-6);
    step_costs(k) = ch(end);

    % 7. Store refined estimates for ALL nodes currently in window
    win_abs = sw.getWindowAbsIndices();
    x_win   = sw.getAllWindowStates();
    for s = 1:length(win_abs)
        x_est(:, win_abs(s)) = x_win(:, s);
    end

    % 8. Store marginal covariance diagonal for the newest node
    P_diag(:, k) = diag(sw.getNewestMarginal());

    % Progress display
    if mod(k, 50) == 0 || k == N_fgo
        fprintf('  step %3d/%d  |  n_active = %2d  |  cost = %.3e\n', ...
                k, N_fgo, sw.n_active, step_costs(k));
    end
end
fprintf('\n');

%% ====================================================================
%  PHASE 4: EXTRACT RESULTS
% =====================================================================
fprintf('Phase 4: Extracting results ...\n');

% Reconstruct pbar from optimised target position
pbar_est = zeros(2, N_fgo);
for k = 1:N_fgo
    p_t_est = x_est(1:3, k);
    M       = R_b2c * R_b2e_fgo(:,:,k)';
    p_cam   = M * (p_t_est - p_int_fgo(:,k));
    if p_cam(3) > 0.01
        pbar_est(:,k) = [p_cam(1)/p_cam(3); p_cam(2)/p_cam(3)];
    end
end

% Package into 8-dim for consistent plotting
x_true_8 = [p_tgt_fgo; v_tgt_fgo; pbar_fgo];
x_est_8  = [x_est(1:3,:); x_est(4:6,:); pbar_est];

%% ====================================================================
%  PHASE 5: PLOT RESULTS
% =====================================================================
fprintf('Phase 5: Plotting ...\n');

% Pass per-step cost as "convergence" curve (cost at last GN iter each step)
plot_swfgo_results(t_fgo, x_true_8, x_est_8, P_diag, p_int_fgo, ...
                   step_costs, W, dt_fgo);

%% ====================================================================
%  PHASE 6: STATISTICS
% =====================================================================
fprintf('\n=== SW-FGO Performance Statistics (W = %d nodes = %.1f s) ===\n', ...
        W, W*dt_fgo);

pos_error = p_tgt_fgo - x_est(1:3, :);
pos_rmse  = sqrt(mean(vecnorm(pos_error,2,1).^2));
fprintf('Target Position RMSE : %.3f m\n', pos_rmse);

vel_error = v_tgt_fgo - x_est(4:6, :);
vel_rmse  = sqrt(mean(vecnorm(vel_error,2,1).^2));
fprintf('Target Velocity RMSE : %.3f m/s\n', vel_rmse);

pbar_error = pbar_fgo - pbar_est;
pbar_rmse  = sqrt(mean(vecnorm(pbar_error,2,1).^2));
fprintf('Image Feature RMSE   : %.4f\n', pbar_rmse);

fprintf('\n=== Final Errors ===\n');
fprintf('Final Position Error : [%.3f, %.3f, %.3f] m\n',   pos_error(:,end)');
fprintf('Final Velocity Error : [%.3f, %.3f, %.3f] m/s\n', vel_error(:,end)');
fprintf('Final pbar Error     : [%.4f, %.4f]\n',           pbar_error(:,end)');
