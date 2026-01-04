%% Error-State Kalman Filter (ESKF) Simulation for Drone Interception
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.
%
% This script demonstrates the ESKF observer for estimating:
% - Interceptor attitude (quaternion via error-state axis-angle)
% - Relative position and velocity
% - Normalized image feature coordinates
% - IMU biases (gyroscope and accelerometer)
%
% ESKF State Structure:
%   Nominal state x (18): [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3)]
%   Error state δx (17): [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]
%
% File Structure:
%   - ErrorStateKalmanFilter.m : ESKF class with prediction/correction methods
%   - compute_eskf_jacobians.m : Error state Jacobian computation
%   - compute_image_features.m : Normalized image coordinate computation
%   - plot_eskf_results.m      : Visualization functions

clear; clc; close all;
rng(32)

%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 1/200;           % IMU update rate: 200 Hz
dt_eskf = 1/200;          % ESKF update rate: 100 Hz (can be different from IMU)
dt_image = 1/30;          % Image update rate: ~30 Hz
dt_radar = 1/0.5;         % RADAR update rate: 0.5 Hz
t_delay = 0/1000;        % Image processing delay: 80 ms
t_total = 25;             % Total simulation time
D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Flag for measurement devices
useRadar = true;
useCam   = true;

% Time vectors
t_imu = 0:dt_imu:t_total;
N_imu = length(t_imu);

%% ======================== PHYSICAL CONSTANTS ========================
g = 9.81;                 % Gravity (m/s^2)
e3 = [0; 0; 1];           % Unit vector in z-direction

%% ======================== CAMERA PARAMETERS ========================
% Camera-to-Body rotation matrix
% Camera z-axis = Body x-axis (forward-looking)
% Camera x-axis = Body y-axis (right)
% Camera y-axis = Body z-axis (down)
R_c2b = [0 0 1;
         1 0 0; 
         0 1 0];          % Transforms vector FROM camera TO body frame
R_b2c = R_c2b';           % Transforms vector FROM body TO camera frame

%% ======================== IMU MODEL (REALISTIC) ========================
% Configure IMU with noise and bias random walk (paper notation)
%
% Units (from paper):
%   σ_ãn  [m/s²]     : Accelerometer measurement noise std
%   σ_ω̃n  [rad/s]    : Gyroscope measurement noise std
%   σ_aw  [m/s²√s]   : Accelerometer bias random walk
%   σ_ωw  [rad/s√s]  : Gyroscope bias random walk
%
% Discrete covariances (computed by IMUModel):
%   V_i = σ²_an * Δt² * I   [m²/s²]     (eq. 262)
%   Θ_i = σ²_ωn * Δt² * I   [rad²]      (eq. 263)
%   A_i = σ²_aw * Δt * I    [m²/s⁴]     (eq. 264)
%   Ω_i = σ²_ωw * Δt * I    [rad²/s²]   (eq. 265)

% === IMU Configuration ===
imu_params = struct();
imu_params.fs = 1/dt_imu;          % Sample rate [Hz] - REQUIRED

% Noise parameters (paper notation)
imu_params.sigma_a_n = 0.1;        % σ_ãn: Accel noise [m/s²] (~10 mg)
imu_params.sigma_omega_n = 0.01;   % σ_ω̃n: Gyro noise [rad/s] (~0.57 deg/s)

% Bias random walk parameters
imu_params.sigma_a_w = 1e-4;       % σ_aw: Accel bias RW [m/s²√s]
imu_params.sigma_omega_w = 1e-5;   % σ_ωw: Gyro bias RW [rad/s√s]

% Initial bias values
imu_params.omega_b_init = [0.005; -0.003; 0.002];  % ω_b(0) [rad/s]
imu_params.a_b_init = [0.02; -0.01; 0.015];        % a_b(0) [m/s²]

% Create IMU model
imu = IMUModel(imu_params);

% Get process noise covariance for ESKF (matches Gc structure)
Qi = imu.getDiscreteProcessNoise(dt_eskf);  % 12x12
Qc = imu.getESKFProcessNoise();

%% ======================== SENSOR NOISE PARAMETERS ========================
% Image sensor noise
sigma_img = 0.005;        % Normalized image coordinate noise (~5 pixels)

% RADAR noise
sigma_radar = 1;          % RADAR position noise: 5 meters

% Measurement noise covariance - Image (2x2)
R_img = sigma_img^2 * eye(2);

% Measurement noise covariance - RADAR (3x3)
R_radar = sigma_radar^2 * eye(3);

%% ======================== INITIAL CONDITIONS ========================
% ===== INTERCEPTOR INITIAL STATE =====
p_int = [0; 0; -65];          % Interceptor position (NED)
v_int = [0; 0; 0];            % Interceptor velocity

% Attitude (level, pointing toward target)
yaw_init = 0;
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

% ===== TARGET INITIAL STATE =====
p_tgt = [50; 10; -40];        % Target position
v_tgt = [0; 0; 0];            % Target velocity

% ===== RELATIVE STATE =====
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

% ===== IMU BIASES (from IMU model, paper notation) =====
% Biases evolve over time according to random walk model
omega_b_true = imu.omega_b;    % ω_b: Gyro bias from IMU model [rad/s]
a_b_true = imu.a_b;            % a_b: Accel bias from IMU model [m/s²]

%% ======================== STATE INDICES (NOMINAL STATE 18-dim) ========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;

%% ======================== ERROR STATE INDICES (17-dim) ========================
idx_dtheta = 1:3;
idx_dpr = 4:6;
idx_dvr = 7:9;
idx_dpbar = 10:11;
idx_dbgyr = 12:14;
idx_dbacc = 15:17;

%% ======================== COMPUTE INITIAL IMAGE FEATURES ========================
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);

%% ======================== INITIALIZE TRUE STATE ========================
x_true = [q_true; p_r_true; v_r_true; pbar_true; omega_b_true; a_b_true];

%% ======================== INITIALIZE ESKF ========================
% Define initial errors (intuitive Euler angles in degrees)
init_errors = struct();
init_errors.euler_deg  = [1; 1; 1];       % [yaw; pitch; roll] errors in degrees
init_errors.position   = [0.5; 0.5; 0.5]; % Position error [m]
init_errors.velocity   = [0.1; -0.05; 0]; % Velocity error [m/s]
init_errors.pbar       = [0.01; -0.02];   % Image feature error
init_errors.b_gyr      = [0 ; 0 ; 0];     % Start with zero bias estimate
init_errors.b_acc      = [0 ; 0 ; 0];     % Start with zero bias estimate

% Create initial nominal state using ESKF static method
x_init = ErrorStateKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                    pbar_true, omega_b_true, a_b_true, ...
                                                    init_errors);

% Define initial covariance standard deviations for ERROR STATE (17x17)
init_sigma = struct();
init_sigma.attitude   = 0.05;     % ~3° attitude uncertainty (radians for δθ)
init_sigma.position   = 3;        % 3m position uncertainty
init_sigma.velocity   = 0.5;      % 0.5 m/s velocity uncertainty
init_sigma.pbar       = 0.1;      % Image feature uncertainty
init_sigma.b_gyr      = 0.005;    % Gyro bias uncertainty (rad/s)
init_sigma.b_acc      = 0.05;     % Accel bias uncertainty (m/s²)

% Create initial 17x17 error covariance using ESKF static method
P_init = ErrorStateKalmanFilter.createInitialCovariance(init_sigma);

% Create ESKF object
eskf_params = struct();
eskf_params.dt_imu = dt_imu;
eskf_params.dt_eskf = dt_eskf;
eskf_params.R_b2c = R_b2c;
eskf_params.Qd = Qi;              
eskf_params.Qc = Qc;  % 12x12 continuous-time process noise
eskf_params.R_img = R_img;
eskf_params.R_radar = R_radar;
eskf_params.x_init = x_init;      % 18x1 nominal state
eskf_params.P_init = P_init;      % 17x17 error covariance
eskf_params.delay_steps = D;

eskf = ErrorStateKalmanFilter(eskf_params);

%% ======================== STORAGE FOR RESULTS ========================
x_true_log = zeros(18, N_imu);    % True nominal state
x_est_log = zeros(18, N_imu);     % Estimated nominal state
P_log = zeros(17, N_imu);         % Error covariance diagonal (17-dim)
omega_b_log = zeros(3, N_imu);    % True gyro bias ω_b (evolving)
a_b_log = zeros(3, N_imu);        % True accel bias a_b (evolving)

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('Starting ESKF Simulation...\n');
fprintf('Total time: %.1f s, IMU rate: %.0f Hz, ESKF rate: %.0f Hz, Image rate: %.0f Hz, RADAR rate: %.1f Hz\n', ...
        t_total, 1/dt_imu, 1/dt_eskf, 1/dt_image, 1/dt_radar);
fprintf('Image delay: %.0f ms (%d IMU cycles)\n\n', t_delay*1000, D);

% ESKF update control
eskf_update_counter = 0;
eskf_sample_idx = round(dt_eskf / dt_imu);
omega_accum = zeros(3, 1);
a_accum = zeros(3, 1);
imu_count = 0;

image_update_counter = 0;
image_sample_idx = round(dt_image / dt_imu);

radar_update_counter = 0;
radar_sample_idx = round(dt_radar / dt_imu);

for k = 1:N_imu
    t = t_imu(k);
    
    %% ==================== TRUE DYNAMICS ====================
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Generate IMU measurements using realistic IMU model
    % (includes evolving biases via random walk and proper noise)
    [omega_meas, a_meas, omega_b_true, a_b_true] = imu.measure(omega_true, a_body_true);
    
    % Update true state with current biases (for accurate error computation)
    x_true(idx_bgyr) = omega_b_true;
    x_true(idx_bacc) = a_b_true;
    
    
    %% ==================== ESKF PREDICTION (at dt_eskf rate) ====================
    % Accumulate IMU measurements
    omega_accum = omega_accum + omega_meas;
    a_accum = a_accum + a_meas;
    imu_count = imu_count + 1;
    
    eskf_update_counter = eskf_update_counter + 1;
    
    if eskf_update_counter >= eskf_sample_idx
        eskf_update_counter = 0;
        
        % Average accumulated IMU measurements
        omega_avg = omega_accum / imu_count;
        a_avg = a_accum / imu_count;
        
        % Run ESKF prediction with averaged measurements
        eskf.predict(omega_avg, a_avg, t);
        
        % Reset accumulators
        omega_accum = zeros(3, 1);
        a_accum = zeros(3, 1);
        imu_count = 0;
    end
    
    %% ==================== ESKF IMAGE CORRECTION (WITH DELAY) ====================
    image_update_counter = image_update_counter + 1;
    
    if image_update_counter >= image_sample_idx && k > D
        image_update_counter = 0;
        
        if useCam
            R_b2e_delayed = quat2rotm(x_true(idx_q)');
            p_r_cam_delayed = R_b2c * R_b2e_delayed' * (-x_true(idx_pr));
            
            if p_r_cam_delayed(3) > 2
                z_meas = [p_r_cam_delayed(1)/p_r_cam_delayed(3); 
                         p_r_cam_delayed(2)/p_r_cam_delayed(3)] + sigma_img * randn(2,1);
                
                eskf.correctImage(z_meas, D);
            end
        end
    end
    
    %% ==================== RADAR CORRECTION (0.5 Hz) ====================
    radar_update_counter = radar_update_counter + 1;
    if useRadar
        if radar_update_counter >= radar_sample_idx
            radar_update_counter = 0;
            
            % Generate RADAR measurement (measures p_r directly)
            p_r_true_current = x_true(idx_pr);
            z_radar = p_r_true_current + sigma_radar * randn(3,1);
            
            % Apply RADAR correction
            eskf.correctRadar(z_radar);
        end
    end
    
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = eskf.x;
    P_log(:, k) = diag(eskf.P);
    omega_b_log(:, k) = omega_b_true;  % Log evolving gyro bias ω_b
    a_b_log(:, k) = a_b_true;          % Log evolving accel bias a_b
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%%\n', 100*k/N_imu);
    end
end

fprintf('\nSimulation Complete!\n');

%% ======================== PLOT RESULTS ========================
plot_eskf_results(t_imu, x_true_log, x_est_log, P_log, ...
                  idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, ...
                  idx_dtheta, idx_dpr, idx_dvr, idx_dpbar, idx_dbgyr, idx_dbacc);

%% ======================== COMPUTE AND DISPLAY STATISTICS ========================
fprintf('\n=== ESKF Performance Statistics ===\n');

% Position RMSE
pos_error = x_true_log(idx_pr, :) - x_est_log(idx_pr, :);
pos_rmse = sqrt(mean(vecnorm(pos_error, 2, 1).^2));
fprintf('Position RMSE: %.3f m\n', pos_rmse);

% Velocity RMSE
vel_error = x_true_log(idx_vr, :) - x_est_log(idx_vr, :);
vel_rmse = sqrt(mean(vecnorm(vel_error, 2, 1).^2));
fprintf('Velocity RMSE: %.3f m/s\n', vel_rmse);

% Attitude error (compute from quaternions)
att_error_deg = zeros(1, N_imu);
for k = 1:N_imu
    q_true_k = x_true_log(idx_q, k);
    q_est_k = x_est_log(idx_q, k);
    % Quaternion error: q_err = q_true * conj(q_est)
    q_err = quatmultiply(q_true_k', quatconj(q_est_k'))';
    % Angle from quaternion error
    att_error_deg(k) = 2 * acosd(min(abs(q_err(1)), 1));
end
att_rmse = sqrt(mean(att_error_deg.^2));
fprintf('Attitude RMSE: %.3f deg\n', att_rmse);

% Gyro bias error
bgyr_error = x_true_log(idx_bgyr, :) - x_est_log(idx_bgyr, :);
bgyr_rmse = sqrt(mean(vecnorm(bgyr_error, 2, 1).^2));
fprintf('Gyro Bias RMSE: %.6f rad/s\n', bgyr_rmse);

% Accel bias error
bacc_error = x_true_log(idx_bacc, :) - x_est_log(idx_bacc, :);
bacc_rmse = sqrt(mean(vecnorm(bacc_error, 2, 1).^2));
fprintf('Accel Bias RMSE: %.6f m/s^2\n', bacc_rmse);

fprintf('\n=== Final Error Statistics ===\n');
fprintf('Final Position Error: [%.3f, %.3f, %.3f] m\n', pos_error(:,end)');
fprintf('Final Velocity Error: [%.3f, %.3f, %.3f] m/s\n', vel_error(:,end)');
fprintf('Final Attitude Error: %.3f deg\n', att_error_deg(end));
