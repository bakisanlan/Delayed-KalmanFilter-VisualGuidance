%% Factor Graph Optimization (FGO) for Target Tracking
% Batch optimisation over camera + radar measurements.
%
% This script mirrors the problem in run_eskf_simulation_reduced.m but
% solves it as a batch Gauss-Newton factor graph optimisation instead of
% a sequential Error-State Kalman Filter.
%
% State per variable node (6-dim):
%   x_k = [p_t(3);  v_t(3)]      target position and velocity
%
% Factors:
%   - Prior:    N(x_0_prior, Sigma_prior)
%   - Dynamics: constant-velocity between factor  x_{k+1} = f(x_k) + w
%   - Camera:   z_cam = pi(R_b2c * R_b2e' * (p_t - p_i)) + n
%   - Radar:    z_rad = [p_t; v_t] + n
%
% Unlike the ESKF, image feature coordinates (pbar) are NOT part of the
% state; they are computed from the optimised target position and known
% interceptor pose.  Measurement delays are trivially handled by attaching
% the camera factor to the correct past variable node.

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));
rng(32)

%% ======================== SIMULATION PARAMETERS ========================
dt_truth  = 1/200;       % Truth propagation rate          [s]
dt_fgo    = 1/10;        % Factor graph node spacing       [s]
dt_cam    = 1/10;        % Camera measurement rate         [s] (sub-sampled)
dt_radar  = 1/0.5;       % Radar measurement rate          [s]
t_total   = 30;          % Total simulation time            [s]
t_delay   = 80/1000;     % Camera processing delay          [s]
D_nodes   = round(t_delay / dt_fgo);   % Delay in FGO nodes

% Time vectors
t_truth = 0:dt_truth:t_total;
N_truth = length(t_truth);

t_fgo   = 0:dt_fgo:t_total;
N_fgo   = length(t_fgo);

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

% Radar RMSE (spherical domain — simplified to isotropic for FGO demo)
radar_params = struct();
radar_params.rmse_range     = 3.0;
radar_params.rmse_azimuth   = deg2rad(1.3);
radar_params.rmse_elevation = deg2rad(3.3);
radar_params.rmse_doppler   = 0.5;
radar_params.dt             = dt_radar;

R_radar = diag([radar_params.rmse_range^2 * ones(1,3), ...
                radar_params.rmse_doppler^2 * ones(1,3)]);

%% ======================== INITIAL CONDITIONS ===========================
p_int = [0; 0; 0];
v_int = [0; 1; -1];
yaw_init  = 0;
q_true    = eul2quat([yaw_init, 0, 0], 'ZYX')';

p_tgt = [160; 200; -50];
radius = 70;
p_tgt_center = p_tgt - [radius; 0; 0];
v_tgt = [0; 0; 0];

% Full truth state (21-dim, compatible with propagate_true_state)
p_r_true  = p_int - p_tgt;
v_r_true  = v_int - v_tgt;
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);
x_true_full = [q_true; p_r_true; v_r_true; pbar_true; ...
               zeros(3,1); zeros(3,1); zeros(3,1)];

% Full-state indices (for propagate_true_state)
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

% Storage at truth rate
p_int_hist  = zeros(3, N_truth);
v_int_hist  = zeros(3, N_truth);
q_hist      = zeros(4, N_truth);
p_tgt_hist  = zeros(3, N_truth);
v_tgt_hist  = zeros(3, N_truth);
pbar_hist   = zeros(2, N_truth);

for k = 1:N_truth
    t = t_truth(k);

    % Store BEFORE propagation (state at time t)
    p_int_hist(:, k)  = p_int;
    v_int_hist(:, k)  = v_int;
    q_hist(:, k)      = x_true_full(idx_q);
    p_tgt_hist(:, k)  = p_tgt;
    v_tgt_hist(:, k)  = v_tgt;
    pbar_hist(:, k)   = x_true_full(idx_pbar_full);

    if k < N_truth
        [x_true_full, p_int, v_int, p_tgt, v_tgt, ~, ~] = ...
            propagate_true_state(x_true_full, p_int, v_int, p_tgt, v_tgt, ...
                                 t, dt_truth, g, e3, R_b2c, ...
                                 idx_q, idx_pr, idx_vr, idx_pbar_full, ...
                                 idx_bgyr, idx_bacc, idx_bmag, p_tgt_center);
    end
end

%% ====================================================================
%  PHASE 2: SUB-SAMPLE TO FGO NODES  &  GENERATE MEASUREMENTS
% =====================================================================
fprintf('Phase 2: Sub-sampling to %d Hz and generating measurements ...\n', ...
        round(1/dt_fgo));

% Sub-sample indices (1-based indices into truth arrays)
fgo_idx = 1 : fgo_sample_ratio : N_truth;
fgo_idx = fgo_idx(1:N_fgo);   % trim if rounding adds an extra point

% Extract truth at FGO nodes
p_tgt_fgo  = p_tgt_hist(:, fgo_idx);
v_tgt_fgo  = v_tgt_hist(:, fgo_idx);
p_int_fgo  = p_int_hist(:, fgo_idx);
v_int_fgo  = v_int_hist(:, fgo_idx);
pbar_fgo   = pbar_hist(:, fgo_idx);

R_b2e_fgo  = zeros(3, 3, N_fgo);
for k = 1:N_fgo
    R_b2e_fgo(:,:,k) = quat2rotm(q_hist(:, fgo_idx(k))');
end

% ----- Camera measurements (10 Hz, at each FGO node) -----
z_cam       = zeros(2, N_fgo);
cam_valid   = false(1, N_fgo);

for k = 1:N_fgo
    M = R_b2c * R_b2e_fgo(:,:,k)';
    p_cam = M * (p_tgt_fgo(:,k) - p_int_fgo(:,k));
    if p_cam(3) > 2
        pbar_k       = [p_cam(1)/p_cam(3); p_cam(2)/p_cam(3)];
        z_cam(:, k)  = pbar_k + sigma_img * randn(2, 1);
        cam_valid(k) = true;
    end
end

% Apply delay: the camera factor for measurement captured at node k
% is attached to node (k - D_nodes) so that it corrects the state at
% the time the image was actually taken.
fprintf('  Camera delay = %.0f ms = %d FGO nodes\n', t_delay*1000, D_nodes);

% ----- Radar measurements (0.5 Hz) -----
radar_node_spacing = round(dt_radar / dt_fgo);
radar_nodes = 1 : radar_node_spacing : N_fgo;

z_radar = zeros(6, length(radar_nodes));
for j = 1:length(radar_nodes)
    k = radar_nodes(j);
    z_radar(:, j) = [p_tgt_fgo(:,k); v_tgt_fgo(:,k)] + ...
                     chol(R_radar, 'lower') * randn(6, 1);
end

fprintf('  %d camera measurements, %d radar measurements\n', ...
        sum(cam_valid), length(radar_nodes));

%% ====================================================================
%  PHASE 3: BUILD FACTOR GRAPH
% =====================================================================
fprintf('Phase 3: Building factor graph with %d variable nodes ...\n', N_fgo);

state_dim = 6;
solver = FactorGraphSolver(N_fgo, state_dim);

% ----- Prior factor on x_0 -----
x0_true = [p_tgt_fgo(:,1); v_tgt_fgo(:,1)];
init_errors = [3; -2; 1.5; 0.3; -0.2; 0.1];    % same as ESKF
x0_prior = x0_true + init_errors;

P_prior = diag([25^2, 25^2, 25^2, 1^2, 1^2, 1^2]);
solver.addPriorFactor(1, x0_prior, P_prior);

% ----- Dynamics factors (constant velocity) -----
% Process noise covariance for constant-velocity model
%   Sigma_dyn = sigma^2 * [dt^4/4*I3, dt^3/2*I3;
%                           dt^3/2*I3, dt^2*I3  ]
dt = dt_fgo;
sigma2 = sigma_tgt_rw^2;
Sigma_dyn = sigma2 * [dt^4/4*eye(3), dt^3/2*eye(3);
                      dt^3/2*eye(3), dt^2*eye(3)];
% Ensure positive definiteness
Sigma_dyn = Sigma_dyn + 1e-10 * eye(6);

for k = 1:N_fgo-1
    solver.addDynamicsFactor(k, k+1, dt, Sigma_dyn);
end

% ----- Camera factors -----
% In batch FGO, measurement delay is handled trivially: the image was
% captured at node k, so we attach the camera factor to node k.
% The processing delay (D_nodes) only affects *when* the data becomes
% available to the processor — irrelevant in batch optimisation since
% all data is collected before solving.
cam_count = 0;
for k = 1:N_fgo
    if ~cam_valid(k), continue; end

    % Attach factor to the node where the image was captured
    solver.addCameraFactor(k, z_cam(:,k), R_img, ...
                           R_b2c, R_b2e_fgo(:,:,k), ...
                           p_int_fgo(:,k));
    cam_count = cam_count + 1;
end

% ----- Radar factors -----
for j = 1:length(radar_nodes)
    k = radar_nodes(j);
    solver.addRadarFactor(k, z_radar(:,j), R_radar);
end

fprintf('  Factors:  1 prior + %d dynamics + %d camera + %d radar = %d total\n', ...
        N_fgo-1, cam_count, length(radar_nodes), solver.n_factors);

%% ====================================================================
%  PHASE 4: INITIALISE STATES  (dead-reckoning from prior)
% =====================================================================
fprintf('Phase 4: Initialising states via dead-reckoning ...\n');

x_init = x0_prior;
for k = 1:N_fgo
    solver.setInitialEstimate(k, x_init);
    % Propagate using constant-velocity model
    p_t = x_init(1:3);
    v_t = x_init(4:6);
    x_init = [p_t + v_t * dt; v_t];
end

%% ====================================================================
%  PHASE 5: OPTIMISE
% =====================================================================
fprintf('\nPhase 5: Running Gauss-Newton optimisation ...\n');
cost_history = solver.optimize(30, 1e-6);
fprintf('\n');

%% ====================================================================
%  PHASE 6: EXTRACT RESULTS
% =====================================================================
fprintf('Phase 6: Extracting results and computing marginal covariances ...\n');

x_est = solver.getAllStates();    % 6 x N_fgo

% Compute marginal covariances
P_marginals = solver.computeAllMarginals();   % 6 x 6 x N_fgo
P_diag = zeros(6, N_fgo);
for k = 1:N_fgo
    P_diag(:, k) = diag(P_marginals(:,:,k));
end

% Reconstruct pbar from optimised target position
pbar_est = zeros(2, N_fgo);
for k = 1:N_fgo
    p_t_est = x_est(1:3, k);
    M = R_b2c * R_b2e_fgo(:,:,k)';
    p_cam = M * (p_t_est - p_int_fgo(:,k));
    if p_cam(3) > 0.01
        pbar_est(:, k) = [p_cam(1)/p_cam(3); p_cam(2)/p_cam(3)];
    end
end

% Package into 8-dim for consistent plotting (match ESKF reduced format)
x_true_8 = [p_tgt_fgo; v_tgt_fgo; pbar_fgo];          % 8 x N_fgo
x_est_8  = [x_est(1:3,:); x_est(4:6,:); pbar_est];    % 8 x N_fgo

%% ====================================================================
%  PHASE 7: PLOT RESULTS
% =====================================================================
fprintf('Phase 7: Plotting ...\n');
plot_fgo_results(t_fgo, x_true_8, x_est_8, P_diag, p_int_fgo, cost_history);

%% ====================================================================
%  PHASE 8: STATISTICS
% =====================================================================
fprintf('\n=== FGO Performance Statistics ===\n');

pos_error = p_tgt_fgo - x_est(1:3, :);
pos_rmse  = sqrt(mean(vecnorm(pos_error, 2, 1).^2));
fprintf('Target Position RMSE: %.3f m\n', pos_rmse);

vel_error = v_tgt_fgo - x_est(4:6, :);
vel_rmse  = sqrt(mean(vecnorm(vel_error, 2, 1).^2));
fprintf('Target Velocity RMSE: %.3f m/s\n', vel_rmse);

pbar_error = pbar_fgo - pbar_est;
pbar_rmse  = sqrt(mean(vecnorm(pbar_error, 2, 1).^2));
fprintf('Image Feature RMSE:   %.4f\n', pbar_rmse);

fprintf('\n=== Final Errors ===\n');
fprintf('Final Position Error: [%.3f, %.3f, %.3f] m\n', pos_error(:,end)');
fprintf('Final Velocity Error: [%.3f, %.3f, %.3f] m/s\n', vel_error(:,end)');
fprintf('Final pbar Error:     [%.4f, %.4f]\n', pbar_error(:,end)');
