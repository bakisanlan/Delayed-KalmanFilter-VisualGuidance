%% Error-State Kalman Filter (ESKF) Simulation for Drone Interception (CLOSED LOOP)
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.
%
% This script demonstrates the CLOSED-LOOP Visual Guidance:
% - Controller uses ESKF estimates to generate control commands.
% - Control commands drive the true interceptor dynamics.
% - Initial states are set such that the target is visible but requires correction.
%

clear; clc; close all;
% rng(32)

%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 1/200;           % IMU update rate: 200 Hz
dt_eskf = 1/200;          % ESKF update rate: 100 Hz (can be different from IMU)
dt_image = 1/30;          % Image update rate: ~30 Hz
dt_radar = 1/0.5;         % RADAR update rate: 0.5 Hz
t_delay = 0/1000;        % Image processing delay: 0 ms for now
t_total = 30;             % Total simulation time (shortened for quick test)
D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Flag for measurement devices
useRadar = true;
useCam   = false;

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

%% ======================== CONTROL CONFIGURATION ========================
% Initialize Controller
ctrl_params = struct();
ctrl_params.Kp_att = 5;
ctrl_params.Kp_acc = 1;
ctrl = InterceptorController(ctrl_params);

%% ======================== VISUALIZATION CONFIGURATION ========================
viz = InterceptorVisualizer([-50 150 -100 100 -100 50]);
dt_viz = 0.05; % Update visualization at 20 Hz
viz_update_counter = 0;
viz_sample_idx = round(dt_viz / dt_imu);

%% ======================== IMU MODEL (REALISTIC) ========================
% Configure IMU with noise and bias random walk (paper notation)
imu_params = struct();
imu_params.fs = 1/dt_imu;          % Sample rate [Hz] - REQUIRED
imu_params.sigma_a_n = 0.1;        % σ_ãn: Accel noise [m/s²] (~10 mg)
imu_params.sigma_omega_n = 0.01;   % σ_ω̃n: Gyro noise [rad/s] (~0.57 deg/s)
imu_params.sigma_a_w = 1e-4;       % σ_aw: Accel bias RW [m/s²√s]
imu_params.sigma_omega_w = 1e-5;   % σ_ωw: Gyro bias RW [rad/s√s]
imu_params.omega_b_init = [0.005; -0.003; 0.002];  % ω_b(0) [rad/s]
imu_params.a_b_init = [0.02; -0.01; 0.015];        % a_b(0) [m/s²]

imu = IMUModel(imu_params);
Qi = imu.getDiscreteProcessNoise(dt_eskf);
Qc = imu.getESKFProcessNoise();

%% ======================== SENSOR NOISE PARAMETERS ========================
sigma_img = 0.005;        % Normalized image coordinate noise (~5 pixels)
sigma_radar = 1;          % RADAR position noise: 1 meters
R_img = sigma_img^2 * eye(2);
R_radar = sigma_radar^2 * eye(3);

%% ======================== INITIAL CONDITIONS ========================
% ===== INTERCEPTOR INITIAL STATE =====
p_int = [0; 0; -65];          % Interceptor position (NED)
v_int = [5; 0; 0];            % Interceptor velocity

% Attitude (Look slightly away from target to test controller)
euler_init = deg2rad([45,20,15]);
q_true = eul2quat(euler_init, 'ZYX')'; 

% ===== TARGET INITIAL STATE =====
p_tgt = [200; 200; -100];        % Target position (same altitude to start)
v_tgt = [0; 0; 0];            % Target velocity

% ===== RELATIVE STATE =====
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

% ===== IMU BIASES =====
omega_b_true = imu.omega_b;
a_b_true = imu.a_b;

%% ======================== STATE INDICES ========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;

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
init_errors = struct();
init_errors.euler_deg  = [1; 1; 1];       % [yaw; pitch; roll] errors in degrees
init_errors.position   = [0.5; 0.5; 0.5]; % Position error [m]
init_errors.velocity   = [0.1; -0.05; 0]; % Velocity error [m/s]
init_errors.pbar       = [0.01; -0.02];   % Image feature error
init_errors.b_gyr      = [0 ; 0 ; 0];     % Start with zero bias estimate
init_errors.b_acc      = [0 ; 0 ; 0];     % Start with zero bias estimate

x_init = ErrorStateKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                    pbar_true, omega_b_true, a_b_true, ...
                                                    init_errors);

init_sigma = struct();
init_sigma.attitude   = 0.05;     
init_sigma.position   = 3;        
init_sigma.velocity   = 0.5;      
init_sigma.pbar       = 0.1;      
init_sigma.b_gyr      = 0.005;    
init_sigma.b_acc      = 0.05;     

P_init = ErrorStateKalmanFilter.createInitialCovariance(init_sigma);

eskf_params = struct();
eskf_params.dt_imu = dt_imu;
eskf_params.dt_eskf = dt_eskf;
eskf_params.R_b2c = R_b2c;
% Qi([4:6,4:6]) = 5*Qi([4:6,4:6]);
% Qi([10:12,10:12]) = 5*Qi([10:12,10:12]);
eskf_params.Qd = Qi;              
eskf_params.Qc = Qc; 
eskf_params.R_img = R_img;
eskf_params.R_radar = R_radar;
eskf_params.x_init = x_init;      
eskf_params.P_init = P_init;      
eskf_params.delay_steps = D;

eskf = ErrorStateKalmanFilter(eskf_params);

%% ======================== STORAGE FOR RESULTS ========================
x_true_log = zeros(18, N_imu);    
x_est_log = zeros(18, N_imu);     
P_log = zeros(17, N_imu);         
omega_cmd_log = zeros(3, N_imu);
a_cmd_log = zeros(3, N_imu);

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('Starting CLOSED-LOOP Simulation...\n');

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

% Initial control command (zero)
omega_cmd = zeros(3,1);
a_cmd = zeros(3,1);

for k = 1:N_imu
    t = t_imu(k);
    
    % --- CONTROLLER STEP ---
    % Use CURRENT ESKF state estimate to generate control
    % Note: eskf.x is best estimate at this time
    q_est = eskf.x(idx_q);
    p_r_est = eskf.x(idx_pr);
    v_r_est = eskf.x(idx_vr);
    
    [omega_cmd, a_cmd] = ctrl.compute_control(q_est, p_r_est, v_r_est);
    
    % Store commands
    omega_cmd_log(:, k) = omega_cmd;
    a_cmd_log(:, k) = a_cmd;

    %% ==================== TRUE DYNAMICS PROPAGATION ====================
    % Apply Control Commands to True Plant
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state_controlled(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             omega_cmd, a_cmd, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Generate IMU measurements 
    [omega_meas, a_meas, omega_b_true, a_b_true] = imu.measure(omega_true, a_body_true);
    
    % Update true state with current biases
    x_true(idx_bgyr) = omega_b_true;
    x_true(idx_bacc) = a_b_true;
    
    
    %% ==================== ESKF PREDICTION ====================
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
    
    %% ==================== MEASUREMENT UPDATES (RADAR & CAMERA) ====================
    % Note: Moved logic inside the updates for cleaner reading if needed, 
    % but keeping structure similar to original.
    
    %% ==================== ESKF IMAGE CORRECTION ====================
    image_update_counter = image_update_counter + 1;
    
    if image_update_counter >= image_sample_idx && k > D
        image_update_counter = 0;
        
        if useCam
            R_b2e_delayed = quat2rotm(x_true(idx_q)');
            p_r_cam_delayed = R_b2c * R_b2e_delayed' * (-x_true(idx_pr));
            
            % Check FOV (p_zc > 0)
            if p_r_cam_delayed(3) > 1 
                z_meas = [p_r_cam_delayed(1)/p_r_cam_delayed(3); 
                         p_r_cam_delayed(2)/p_r_cam_delayed(3)] + sigma_img * randn(2,1);
                
                eskf.correctImage(z_meas, D);
            end
        end
    end
    
    %% ==================== RADAR CORRECTION ====================
    radar_update_counter = radar_update_counter + 1;
    if useRadar
        if radar_update_counter >= radar_sample_idx
            radar_update_counter = 0;
            
            % Generate RADAR measurement
            p_r_true_current = x_true(idx_pr);
            z_radar = p_r_true_current + sigma_radar * randn(3,1);
            
            eskf.correctRadar(z_radar);
        end
    end
    
    %% ==================== VISUALIZATION UPDATE ====================
    viz_update_counter = viz_update_counter + 1;
    if viz_update_counter >= viz_sample_idx
        viz_update_counter = 0;
        
        % Extract states for visualization
        % True
        p_int_vis = p_int;
        q_true_vis = x_true(idx_q);
        p_tgt_vis = p_tgt;
        
        % Estimated
        q_est_vis = eskf.x(idx_q);
        p_r_est_vis = eskf.x(idx_pr);
        
        % Derived Estimated Target Position (relative to True Interceptor)
        % Since we don't have inertial estimated interceptor position,
        % we display where the estimator thinks the target is relative to the
        % true interceptor position.
        % p_tgt_est = p_int_true - p_r_est
        p_tgt_est_vis = p_int_vis - p_r_est_vis;

        disp(norm(x_true(idx_pr)))
        disp(norm(v_tgt))
        
        viz.update(p_int_vis, q_true_vis, q_est_vis, p_tgt_vis, p_tgt_est_vis);
        drawnow limitrate;
    end
    
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = eskf.x;
    P_log(:, k) = diag(eskf.P);
    
    %% ==================== CHECK FOR IMPACT ====================
    if norm(x_true(idx_pr)) < 5.0
        fprintf('\nIMPACT DETECTED at t = %.3f s! Distance: %.3f m\n', t, norm(x_true(idx_pr)));
        
        % Truncate logs to actual duration
        x_true_log = x_true_log(:, 1:k);
        x_est_log = x_est_log(:, 1:k);
        P_log = P_log(:, 1:k);
        omega_cmd_log = omega_cmd_log(:, 1:k);
        a_cmd_log = a_cmd_log(:, 1:k);
        t_imu = t_imu(1:k);
        
        break;
    end
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%%\n', 100*k/N_imu);
    end
end

fprintf('\nSimulation Complete!\n');

%% ======================== PLOT RESULTS ========================
% Plot standard ESKF results
plot_eskf_results(t_imu, x_true_log, x_est_log, P_log, ...
                  idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, ...
                  idx_dtheta, idx_dpr, idx_dvr, idx_dpbar, idx_dbgyr, idx_dbacc);

% Plot Control Commands
figure('Name', 'Control Commands', 'Color', 'w');
subplot(2,1,1);
plot(t_imu, omega_cmd_log); grid on;
title('Commanded Angular Velocity');
ylabel('rad/s'); legend('x','y','z');

subplot(2,1,2);
plot(t_imu, a_cmd_log); grid on;
title('Commanded Specific Force');
ylabel('m/s^2'); legend('x','y','z');
xlabel('Time [s]');
