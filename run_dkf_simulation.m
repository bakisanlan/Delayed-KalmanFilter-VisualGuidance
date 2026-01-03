%% Delayed Kalman Filter (DKF) Simulation for Drone Interception
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.
%
% This script demonstrates the DKF observer for estimating:
% - Interceptor attitude (quaternion)
% - Relative position and velocity
% - Normalized image feature coordinates
% - IMU biases (gyroscope and accelerometer)
%
% File Structure:
%   - DelayedKalmanFilter.m : DKF class with prediction/correction methods
%   - compute_jacobians.m   : State transition and noise Jacobian computation
%   - compute_image_features.m : Normalized image coordinate computation
%   - plot_dkf_results.m    : Visualization functions

clear; clc; close all;
rng(31)
%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 1/200;           % IMU update rate: 200 Hz
dt_image = 1/30;          % Image update rate: ~30 Hz
dt_radar = 1/0.5;             % RADAR update rate: 1 Hz
t_delay = 0/1000;              % Image processing delay: 80 ms
t_total = 10;             % Total simulation time
D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Flag for measurement devices
useRadar =true;
useCam   =true;

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

%% ======================== NOISE PARAMETERS ========================
% Commercial MEMS IMU specifications
sigma_gyr = 0.003;        % Gyroscope noise: 0.003 rad/s (~0.17 deg/s)
sigma_acc = 0.03;         % Accelerometer noise: 0.03 m/s²
sigma_bgyr = 5e-6;        % Gyroscope bias random walk
sigma_bacc = 5e-5;        % Accelerometer bias random walk

% Image sensor noise
sigma_img = 0.005;        % Normalized image coordinate noise (~5 pixels)

% RADAR noise
sigma_radar = 5;        % RADAR position noise: 1 meter

% Process noise covariance Q (6x6 for bias random walks)
Q = diag([sigma_bgyr^2 * ones(1,3), sigma_bacc^2 * ones(1,3)]) * dt_imu;
% Q = diag([sigma_gyr^2 * ones(1,3), sigma_acc^2 * ones(1,3)]) * dt_imu;


% Measurement noise covariance - Image (2x2)
R_img = sigma_img^2 * eye(2);

% Measurement noise covariance - RADAR (3x3)
R_radar = sigma_radar^2 * eye(3);

%% ======================== INITIAL CONDITIONS ========================
% ===== INTERCEPTOR INITIAL STATE =====
p_int = [0; 0; -65];          % Interceptor position (NED)
v_int = [0; 0; 0];            % Interceptor velocity

% Attitude (level, pointing toward target)
yaw_init = 0; %atan2(10, 30);
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

% ===== TARGET INITIAL STATE =====
p_tgt = [30; 5; -40];        % Target position
v_tgt = [0; 0; 0];            % Target velocity

% ===== RELATIVE STATE =====
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

% ===== IMU BIASES (typical values for commercial IMU) =====
b_gyr_true = [0.005; -0.003; 0.002];   % ~0.3 deg/s
b_acc_true = [0.02; -0.01; 0.015];     % m/s²

%% ======================== STATE INDICES ========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;

%% ======================== COMPUTE INITIAL IMAGE FEATURES ========================
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);

%% ======================== INITIALIZE TRUE STATE ========================
x_true = [q_true; p_r_true; v_r_true; pbar_true; b_gyr_true; b_acc_true];

%% ======================== INITIALIZE DKF ========================
% Define initial errors (intuitive Euler angles in degrees)
init_errors = struct();
init_errors.euler_deg  = [1; 1; 1];      % [yaw; pitch; roll] errors in degrees
init_errors.position   = [0.5; 0.5; 0.5];     % Position error [m]
init_errors.velocity   = [0.1; -0.05; 0];  % Velocity error [m/s]
init_errors.pbar       = [0.01; -0.02];        % Image feature error
init_errors.b_gyr      = 0;        % Start with zero bias estimate
init_errors.b_acc      = 0;         % Start with zero bias estimate

% Create initial state using DKF static method
x_init = DelayedKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                 pbar_true, b_gyr_true, b_acc_true, ...
                                                 init_errors);

% Define initial covariance standard deviations (commercial IMU)
init_sigma = struct();
init_sigma.quaternion = 0.05;   % ~3° attitude uncertainty
init_sigma.position   = 3;        % 3m position uncertainty
init_sigma.velocity   = 0.5;      % 0.5 m/s velocity uncertainty
init_sigma.pbar       = 0.1;          % Image feature uncertainty
init_sigma.b_gyr      = 0.005;       % Gyro bias uncertainty (rad/s)
init_sigma.b_acc      = 0.05;        % Accel bias uncertainty (m/s²)

% Create initial covariance using DKF static method
P_init = DelayedKalmanFilter.createInitialCovariance(init_sigma);

% Create DKF object
dkf_params = struct();
dkf_params.dt_imu = dt_imu;
dkf_params.R_b2c = R_b2c;
dkf_params.Q = Q;
dkf_params.R_img = R_img;
dkf_params.R_radar = R_radar;
dkf_params.x_init = x_init;
dkf_params.P_init = P_init;
dkf_params.delay_steps = D;

dkf = DelayedKalmanFilter(dkf_params);

%% ======================== STORAGE FOR RESULTS ========================
x_true_log = zeros(18, N_imu);
x_est_log = zeros(18, N_imu);
P_log = zeros(18, N_imu);

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('Starting DKF Simulation...\n');
fprintf('Total time: %.1f s, IMU rate: %.0f Hz, Image rate: %.0f Hz, RADAR rate: %.0f Hz\n', ...
        t_total, 1/dt_imu, 1/dt_image, 1/dt_radar);
fprintf('Image delay: %.0f ms (%d IMU cycles)\n\n', t_delay*1000, D);

image_update_counter = 0;
image_sample_idx = round(dt_image / dt_imu);

radar_update_counter = 0;
radar_sample_idx = round(dt_radar / dt_imu);  % RADAR at 1 Hz

for k = 1:N_imu
    t = t_imu(k);
    
    %% ==================== TRUE DYNAMICS ====================
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Generate IMU measurements (add bias and noise)
    [omega_meas, a_meas] = generate_imu_measurements(omega_true, a_body_true, ...
                                                      b_gyr_true, b_acc_true, ...
                                                      sigma_gyr, sigma_acc);
    tic
    %% ==================== DKF PREDICTION ====================
    dkf.predict(omega_meas, a_meas, t);
    a = x_true - dkf.x;

    %% ==================== DKF CORRECTION (WITH DELAY) ====================
    image_update_counter = image_update_counter + 1;
    
    if image_update_counter >= image_sample_idx && k > D
        image_update_counter = 0;
        
        % Simulate delayed image measurement (currently disabled)
        if useCam  % Enable when ready to test correction
            R_b2e_delayed = quat2rotm(x_true(idx_q)');
            p_r_cam_delayed = R_b2c * R_b2e_delayed' * (-x_true(idx_pr));
            
            if p_r_cam_delayed(3) > 2
                z_meas = [p_r_cam_delayed(1)/p_r_cam_delayed(3); 
                         p_r_cam_delayed(2)/p_r_cam_delayed(3)] + sigma_img * randn(2,1);
                
                dkf.correct(z_meas, D);
            end
        end
    end
    
    %% ==================== RADAR CORRECTION (1 Hz) ====================
    radar_update_counter = radar_update_counter + 1;
    if useRadar
        if radar_update_counter >= radar_sample_idx
            radar_update_counter = 0;
            
            % Generate RADAR measurement (measures p_r directly)
            p_r_true = x_true(idx_pr);
            z_radar = p_r_true + sigma_radar * randn(3,1);
            
            % Apply RADAR correction
            dkf.correctRadar(z_radar);
        end
    end
    toc
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = dkf.x;
    P_log(:, k) = diag(dkf.P);
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%%\n', 100*k/N_imu);
    end
end

fprintf('\nSimulation Complete!\n');

%% ======================== PLOT RESULTS ========================
plot_dkf_results(t_imu, x_true_log, x_est_log, P_log, ...
                 idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
