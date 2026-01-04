%% Delayed Kalman Filter (DKF) Simulation for Drone Interception
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.
%
% This script implements the DKF observer for estimating:
% - Interceptor attitude (quaternion)
% - Relative position and velocity
% - Normalized image feature coordinates
% - IMU biases (gyroscope and accelerometer)
%
% Scenario: Static target hovering

clear; clc; close all;

%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 0.005;           % IMU update rate: 200 Hz
dt_image = 0.033;         % Image update rate: ~30 Hz
t_delay = 0.080;          % Image processing delay: 80 ms
t_total = 9.80;             % Total simulation time: 10 seconds
D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Time vectors
t_imu = 0:dt_imu:t_total;
N_imu = length(t_imu);

%% ======================== PHYSICAL CONSTANTS ========================
g = 9.81;                 % Gravity (m/s^2)
e3 = [0; 0; 1];           % Unit vector in z-direction # CHECK SIGN

%% ======================== CAMERA PARAMETERS ========================
% Camera intrinsics (example values)
fx = 500;                 % Focal length x (pixels)
fy = 500;                 % Focal length y (pixels)
cx = 320;                 % Principal point x (pixels)
cy = 240;                 % Principal point y (pixels)
f_oc = fx;                % Focal length for normalization

% Camera-to-Body rotation matrix
% Camera z-axis = Body x-axis (forward-looking)
% Camera x-axis = Body y-axis (right)
% Camera y-axis = Body z-axis (down)
R_c2b = [0 0 1;   % NOTE CHANGED, TOOK TRANSPOSE
         1 0 0; 
         0 1 0];          % Transforms vector FROM camera TO body frame
R_b2c = R_c2b';           % Transforms vector FROM body TO camera frame

%% ======================== NOISE PARAMETERS ========================
% IMU noise (continuous-time spectral densities)
sigma_gyr = 0.0001;         % Gyroscope noise (rad/s)  % TODO
sigma_acc = 0.001;          % Accelerometer noise (m/s^2)
sigma_bgyr = 1e-6;        % Gyroscope bias random walk
sigma_bacc = 1e-5;        % Accelerometer bias random walk

% Image measurement noise
sigma_img = 0.002;        % Normalized image coordinate noise

%% ======================== PROCESS & MEASUREMENT NOISE ========================
% Process noise covariance Q (6x6 for bias noises)
Q = diag([sigma_bgyr^2 * ones(1,3), sigma_bacc^2 * ones(1,3)]) * dt_imu;   % TODO

% Measurement noise covariance R (2x2)
R = sigma_img^2 * eye(2);

%% ======================== INITIAL CONDITIONS ========================
% ===== INTERCEPTOR INITIAL STATE =====
% Position and velocity in EFCS (Earth-Fixed Coordinate System)
p_int = [0; 0; -40];          % Interceptor position (NED: z negative = up)
v_int = [0; 0; 0];            % Interceptor velocity

% True attitude (quaternion [q0, q1, q2, q3] where q0 is scalar)
% Initially level, pointing roughly toward target
yaw_init = atan2(10, 30);     % Approximate direction to target
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';  % [q0; q1; q2; q3]

% ===== TARGET INITIAL STATE =====
p_tgt = [30; 10; -50];        % Target position
v_tgt = [0; 0; 0];            % Target velocity (static initially)
a_tgt = [0; 0; 0];            % Target acceleration (for maneuvering target)

% ===== RELATIVE STATE (computed from interceptor and target) =====
p_r_true = p_int - p_tgt;     % Relative position
v_r_true = v_int - v_tgt;     % Relative velocity

% True IMU biases
b_gyr_true = [0.01; -0.005; 0.002];   % True gyroscope bias (rad/s)  % TODO
b_acc_true = [0.05; -0.03; 0.02];     % True accelerometer bias (m/s^2)

%% ======================== STATE VECTOR DEFINITION ========================
% State vector x (18x1):
% x = [q(4), p_r(3), v_r(3), p_bar(2), b_gyr(3), b_acc(3)]^T
%
% Indices:
idx_q = 1:4;              % Quaternion
idx_pr = 5:7;             % Relative position
idx_vr = 8:10;            % Relative velocity
idx_pbar = 11:12;         % Normalized image coordinates
idx_bgyr = 13:15;         % Gyroscope bias
idx_bacc = 16:18;         % Accelerometer bias

%% ======================== INITIALIZE TRUE STATE ========================
% Compute true normalized image coordinates
R_b2e_true = quat2rotm(q_true');      % Rotation from body to EFCS
p_r_cam = R_b2c * R_b2e_true' * (-p_r_true);  % Position of target in camera frame
p_zc_true = p_r_cam(3);               % Depth in camera frame
pbar_true = [p_r_cam(1)/p_r_cam(3); p_r_cam(2)/p_r_cam(3)];  % Normalized coords  % TODO

% True state vector (relative states)
x_true = [q_true; p_r_true; v_r_true; pbar_true; b_gyr_true; b_acc_true];

%% ======================== INITIALIZE ESTIMATED STATE ========================
% Add initial estimation errors
q_est_init = q_true + 0*[0; 0.02; -0.01; 0.015];  %TODO
q_est_init = q_est_init / norm(q_est_init);  % Normalize quaternion

p_r_est_init = p_r_true + 0*[2; -1; 0.5];      % Position error
v_r_est_init = v_r_true + 0*[0.1; -0.05; 0.02]; % Velocity error
pbar_est_init = pbar_true + 0*[0.01; -0.02];   % Image feature error
b_gyr_est_init = b_gyr_true; %[0; 0; 0];                  % Unknown biases
b_acc_est_init = b_acc_true;%[0; 0; 0];

% Initial estimated state
x_est = [q_est_init; p_r_est_init; v_r_est_init; pbar_est_init; ...
         b_gyr_est_init; b_acc_est_init];

%% ======================== INITIALIZE COVARIANCE ========================
P_init = diag([0.01*ones(1,4), ...     % Quaternion uncertainty
               4*ones(1,3), ...         % Position uncertainty (m^2)
               0.1*ones(1,3), ...       % Velocity uncertainty (m^2/s^2)
               0.01*ones(1,2), ...      % Image feature uncertainty
               0.001*ones(1,3), ...      % Gyro bias uncertainty
               0.0001*ones(1,3)]);         % Accel bias uncertainty
P = P_init;

%% ======================== STORAGE FOR HISTORY ========================
% For delay handling, we need to store past states and covariances
history_length = D + 10;  % Store extra for safety
x_history = zeros(18, history_length);
P_history = zeros(18, 18, history_length);
imu_history = zeros(6, history_length);  % [omega; accel]
time_history = zeros(1, history_length);

% Initialize history with initial estimate
for i = 1:history_length
    x_history(:, i) = x_est;
    P_history(:, :, i) = P;
    time_history(i) = -history_length * dt_imu + (i-1) * dt_imu;
end

%% ======================== STORAGE FOR RESULTS ========================
x_true_log = zeros(18, N_imu);
x_est_log = zeros(18, N_imu);
P_log = zeros(18, N_imu);  % Store diagonal of P
z_meas_log = zeros(2, N_imu);
innovation_log = zeros(2, N_imu);

% Additional storage for interceptor and target absolute states
p_int_log = zeros(3, N_imu);
v_int_log = zeros(3, N_imu);
p_tgt_log = zeros(3, N_imu);
v_tgt_log = zeros(3, N_imu);

%% ======================== MEASUREMENT MATRIX H ========================
H = zeros(2, 18);
H(1:2, idx_pbar) = eye(2);

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('Starting DKF Simulation...\n');
fprintf('Total time: %.1f s, IMU rate: %.0f Hz, Image rate: %.0f Hz\n', ...
        t_total, 1/dt_imu, 1/dt_image);
fprintf('Image delay: %.0f ms (%d IMU cycles)\n\n', t_delay*1000, D);

image_update_counter = 0;
image_sample_idx = round(dt_image / dt_imu);

for k = 1:N_imu
    t = t_imu(k);
    
    %% ==================== TRUE DYNAMICS (SIMULATION) ====================
    % ===== INTERCEPTOR DYNAMICS =====
    % True angular velocity (small oscillations for realism)
    omega_true = 0.001 * sin(0.5 * t) * [1; 0.5; 0.2];
    
    % True specific force (gravity compensation + maneuver acceleration)
    R_b2e_true = quat2rotm(x_true(idx_q)');
    a_body_true = R_b2e_true' * (-g * e3) + sin(t) * [3; 3; 1];  % Body-frame accel
    
    % Interceptor acceleration in world frame
    a_int_world = R_b2e_true * a_body_true + g * e3;
    
    % Propagate interceptor quaternion
    omega_quat = [0; omega_true];
    dq = 0.5 * quatmultiply(q_true', omega_quat')';
    q_true = q_true + dq * dt_imu;
    q_true = q_true / norm(q_true);
    
    % Propagate interceptor velocity and position
    v_int_new = v_int + a_int_world * dt_imu;
    p_int = p_int + 0.5 * (v_int + v_int_new) * dt_imu;
    v_int = v_int_new;
    
    % ===== TARGET DYNAMICS =====
    % Target motion model (constant velocity with optional acceleration)
    % You can modify this for different target behaviors:
    % - Static target: a_tgt = [0; 0; 0]
    % - Constant velocity: a_tgt = [0; 0; 0], v_tgt = constant
    % - Maneuvering: a_tgt = some function of time
    
    % Example: Hovering target with small oscillations
    a_tgt = 0.1 * sin(0.3 * t) * [0.1; 0.1; 0];  % Small perturbations
    
    % Propagate target velocity and position
    v_tgt_new = v_tgt + a_tgt * dt_imu;
    p_tgt = p_tgt + 0.5 * (v_tgt + v_tgt_new) * dt_imu;
    v_tgt = v_tgt_new;
    
    % ===== COMPUTE RELATIVE STATE =====
    p_r_true = p_int - p_tgt;     % Relative position
    v_r_true = v_int - v_tgt;     % Relative velocity
    
    % Compute image features from relative position
    R_b2e_true = quat2rotm(q_true');
    p_r_cam = R_b2c * R_b2e_true' * (-p_r_true);
    if p_r_cam(3) > 0.1
        pbar_true = [p_r_cam(1)/p_r_cam(3); p_r_cam(2)/p_r_cam(3)];
    else
        pbar_true = x_true(idx_pbar);  % Keep previous if target behind camera
    end
    
    % Assemble true state vector
    x_true = [q_true; p_r_true; v_r_true; pbar_true; b_gyr_true; b_acc_true];
    
    % ===== GENERATE IMU MEASUREMENTS =====
    omega_meas = omega_true + b_gyr_true + sigma_gyr * randn(3,1);
    a_meas = a_body_true + b_acc_true + sigma_acc * randn(3,1);
    
    %% ==================== DKF PREDICTION STEP ====================
    % Extract current estimates
    q_est = x_est(idx_q);
    p_r_est = x_est(idx_pr);
    v_r_est = x_est(idx_vr);
    pbar_est = x_est(idx_pbar);
    b_gyr_est = x_est(idx_bgyr);
    b_acc_est = x_est(idx_bacc);
    
    % Corrected angular velocity and acceleration
    omega_corrected = omega_meas - b_gyr_est;
    a_corrected = a_meas - b_acc_est;
    
    % Get rotation matrix
    R_b2e = quat2rotm(q_est');
    
    % Compute p_zc (depth in camera frame)
    p_r_cam = R_b2c * R_b2e' * (-p_r_est);
    p_zc = max(p_r_cam(3), 0.1);  % Prevent division by zero
    
    % Compute camera-frame velocities
    v_cam = R_b2c * R_b2e' * v_r_est;
    omega_cam = R_b2c * omega_corrected;
    
    % Predict state
    [x_pred] = predict_state(x_est, omega_corrected, a_corrected, dt_imu, g, e3, ...
                             R_b2c, R_b2e, p_zc, idx_q, idx_pr, idx_vr, idx_pbar);
    
    % Compute Jacobians F and G
    [F, G] = compute_jacobians(x_est, omega_corrected, a_corrected, dt_imu, ...
                               R_b2c, R_b2e, p_zc, v_r_est, pbar_est, ...
                               idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Predict covariance
    P_pred = F * P * F' + G * Q * G';
    
    % Update state and covariance
    x_est = x_pred;
    P = P_pred;
    
    % Normalize quaternion
    x_est(idx_q) = x_est(idx_q) / norm(x_est(idx_q));
    
    %% ==================== UPDATE HISTORY BUFFER ====================
    % Shift history buffer
    x_history(:, 1:end-1) = x_history(:, 2:end);
    P_history(:, :, 1:end-1) = P_history(:, :, 2:end);
    imu_history(:, 1:end-1) = imu_history(:, 2:end);
    time_history(1:end-1) = time_history(2:end);
    
    % Add current state to history
    x_history(:, end) = x_est;
    P_history(:, :, end) = P;
    imu_history(:, end) = [omega_meas; a_meas];
    time_history(end) = t;
    
    %% ==================== DKF CORRECTION STEP (WITH DELAY) ====================
    image_update_counter = image_update_counter + 1;
    
    if image_update_counter >= image_sample_idx && k > D
        image_update_counter = 0;
        
        % Get delayed measurement (image was captured D steps ago)
        % Simulate the delayed image measurement using TRUE state from D steps ago
        % In reality, this comes from the camera
        
        % Find the state D steps ago in history
        idx_delayed = history_length - D;
        
        % if idx_delayed >= 1
        if true    
        % True image measurement (what the camera would have seen D steps ago)
            % Use current true state for simplicity (in practice use buffered true state)
            R_b2e_delayed = quat2rotm(x_true(idx_q)');  % TODO
            p_r_cam_delayed = R_b2c * R_b2e_delayed' * (-x_true(idx_pr));  % TODO
            
            if p_r_cam_delayed(3) > 0.1  % Target in front of camera  
                z_meas = [p_r_cam_delayed(1)/p_r_cam_delayed(3);  % TODO
                         p_r_cam_delayed(2)/p_r_cam_delayed(3)] + sigma_img * randn(2,1);
                
                % Get prior state estimate from D steps ago
                x_prior = x_history(:, idx_delayed);
                P_prior = P_history(:, :, idx_delayed);
                
                % Kalman gain
                S = H * P_prior * H' + R;
                K = P_prior * H' / S;
                
                % Innovation
                z_pred = H * x_prior;
                innovation = z_meas - z_pred;
                
                % Correct the delayed state
                x_corrected = x_prior + K * innovation;
                P_corrected = (eye(18) - K * H) * P_prior;
                
                % Normalize quaternion
                x_corrected(idx_q) = x_corrected(idx_q) / norm(x_corrected(idx_q));
                
                % Re-propagate from corrected delayed state to current time
                x_reprop = x_corrected;
                P_reprop = P_corrected;
                
                for j = idx_delayed+1:history_length
                    % Get IMU data from history
                    omega_hist = imu_history(1:3, j);
                    a_hist = imu_history(4:6, j);
                    
                    % Correct with bias estimates
                    omega_corr = omega_hist - x_reprop(idx_bgyr);
                    a_corr = a_hist - x_reprop(idx_bacc);
                    
                    % Get rotation
                    R_b2e_j = quat2rotm(x_reprop(idx_q)');
                    
                    % Compute p_zc
                    p_r_cam_j = R_b2c * R_b2e_j' * (-x_reprop(idx_pr));
                    p_zc_j = max(p_r_cam_j(3), 0.1);
                    
                    % Propagate state
                    [x_reprop] = predict_state(x_reprop, omega_corr, a_corr, dt_imu, g, e3, ...
                                               R_b2c, R_b2e_j, p_zc_j, idx_q, idx_pr, idx_vr, idx_pbar);
                    
                    % Compute Jacobians and propagate covariance
                    [F_j, G_j] = compute_jacobians(x_reprop, omega_corr, a_corr, dt_imu, ...
                                                   R_b2c, R_b2e_j, p_zc_j, x_reprop(idx_vr), ...
                                                   x_reprop(idx_pbar), idx_q, idx_pr, idx_vr, ...
                                                   idx_pbar, idx_bgyr, idx_bacc);
                    P_reprop = F_j * P_reprop * F_j' + G_j * Q * G_j';
                    
                    % Normalize quaternion
                    x_reprop(idx_q) = x_reprop(idx_q) / norm(x_reprop(idx_q));
                end
                
                % Update current state with re-propagated values
                x_est = x_reprop;
                P = P_reprop;
                
                % Log innovation
                innovation_log(:, k) = innovation;
                z_meas_log(:, k) = z_meas;
            end
        end
    end
    
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = x_est;
    P_log(:, k) = diag(P);
    
    % Store absolute states for debugging
    p_int_log(:, k) = p_int;
    v_int_log(:, k) = v_int;
    p_tgt_log(:, k) = p_tgt;
    v_tgt_log(:, k) = v_tgt;
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%%\n', 100*k/N_imu);
    end
end

fprintf('\nSimulation Complete!\n');

%% ======================== PLOT RESULTS ========================
plot_results(t_imu, x_true_log, x_est_log, P_log, idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);

%% ======================== HELPER FUNCTIONS ========================

function [x_pred] = predict_state(x, omega, a_body, dt, g, e3, R_b2c, R_b2e, p_zc, ...
                                  idx_q, idx_pr, idx_vr, idx_pbar)
    % DKF state prediction using IMU measurements
    q = x(idx_q);
    p_r = x(idx_pr);
    v_r = x(idx_vr);
    pbar = x(idx_pbar);
    
    % Quaternion prediction using M(delta_q) matrix
    wx = omega(1); wy = omega(2); wz = omega(3);
    M_dq = [1,        -wx*dt/2, -wy*dt/2, -wz*dt/2;  % CHECKED: TRUE
            wx*dt/2,   1,        wz*dt/2, -wy*dt/2;
            wy*dt/2,  -wz*dt/2,  1,        wx*dt/2;
            wz*dt/2,   wy*dt/2, -wx*dt/2,  1];
    q_pred = M_dq * q;
    q_pred = q_pred / norm(q_pred);
    
    % Velocity prediction
    a_world = R_b2e * a_body + g * e3;
    v_r_pred = v_r + a_world * dt;
    
    % Position prediction
    p_r_pred = p_r + 0.5 * (v_r + v_r_pred) * dt;
    
    % Image feature prediction (Eq. in paper)
    pbar_x = pbar(1); pbar_y = pbar(2);
    
    % Linear velocity contribution
    Lv = [-1/p_zc,     0,    pbar_x/p_zc;  % CHECKED: TRUE
           0,     -1/p_zc,   pbar_y/p_zc];
    v_cam = R_b2c * R_b2e' * v_r;
    
    % Angular velocity contribution
    Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;  % CHECKED: TRUE
          (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
    omega_cam = R_b2c * omega;

    pbar_pred = pbar + (Lv * v_cam + Lw * omega_cam) * dt;
    
    % Biases remain constant (random walk handled in covariance)
    x_pred = x;
    x_pred(idx_q) = q_pred;
    x_pred(idx_pr) = p_r_pred;
    x_pred(idx_vr) = v_r_pred;
    x_pred(idx_pbar) = pbar_pred;
end

function [F, G] = compute_jacobians(x, omega, a_body, dt, R_b2c, R_b2e, p_zc, v_r, pbar, ...
                                    idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc)
    % Compute state transition Jacobian F (18x18) and process noise Jacobian G (18x6)
    
    q = x(idx_q);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    pbar_x = pbar(1); pbar_y = pbar(2);
    
    wx = omega(1); wy = omega(2); wz = omega(3);
    ax = a_body(1); ay = a_body(2); az = a_body(3);
    
    % Initialize F
    F = eye(18);
    
    %% F_q_q: Quaternion to quaternion (M(delta_q))
    F(1:4, 1:4) = [1,        -wx*dt/2, -wy*dt/2, -wz*dt/2; % CHECKED: TRUE
                   wx*dt/2,   1,        wz*dt/2, -wy*dt/2;
                   wy*dt/2,  -wz*dt/2,  1,        wx*dt/2;
                   wz*dt/2,   wy*dt/2, -wx*dt/2,  1];
    
    %% F_q_bgyr: Quaternion to gyro bias
    F_q_bgyr = [q1/2,   q2/2,   q3/2;     % CHECKED: TRUE
               -q0/2,   q3/2,  -q2/2;
               -q3/2,  -q0/2,   q1/2;
                q2/2,  -q1/2,  -q0/2] * dt;
    F(1:4, idx_bgyr) = F_q_bgyr;
    
    %% F_pr_vr: Position to velocity
    F(idx_pr, idx_vr) = eye(3) * dt;
    
    %% F_vr_q: Velocity to quaternion
    % M1, M2, M3 matrices from paper
    M1 = [q0, -q3,  q2;   % CHECKED: TRUE
          q1,  q2,  q3;
         -q2,  q1,  q0;
         -q3, -q0,  q1];
    M2 = [q3,  q0, -q1;   % CHECKED: TRUE
          q2, -q1, -q0;
          q1,  q2,  q3;
          q0, -q3,  q2];
    M3 = [-q2,  q1,  q0;  % CHECKED: TRUE
           q3,  q0, -q1;
          -q0,  q3, -q2;
           q1,  q2,  q3];
    
    F_vr_q = 2 * [M1 * a_body, M2 * a_body, M3 * a_body]' * dt;  % CHECKED: TRUE
    F(idx_vr, idx_q) = F_vr_q;
    
    %% F_vr_bacc: Velocity to accelerometer bias
    F(idx_vr, idx_bacc) = -R_b2e * dt;  % CHECKED: TRUE
    
    %% F_pbar_q: Image feature to quaternion
    % M4 and M5 matrices from paper
    M4 = [2*pbar_x*q0+2*q3,   2*pbar_x*q3-2*q0,  -2*pbar_x*q2-2*q1;  % CHECKED: TRUE
          2*pbar_x*q1-2*q2,   2*pbar_x*q2+2*q1,   2*pbar_x*q3-2*q0;
          2*pbar_x*q2-2*q1,   2*pbar_x*q1-2*q2,  -2*pbar_x*q0-2*q3;
          2*pbar_x*q3+2*q0,   2*pbar_x*q0+2*q3,   2*pbar_x*q1-2*q2];
    
    M5 = [2*pbar_y*q0-2*q2,   2*pbar_y*q3+2*q1,  -2*pbar_y*q2-2*q0; % CHECKED: TRUE
          2*pbar_y*q1-2*q3,   2*pbar_y*q2+2*q0,   2*pbar_y*q3+2*q1;
          2*pbar_y*q2-2*q0,   2*pbar_y*q1-2*q3,  -2*pbar_y*q0+2*q2;
          2*pbar_y*q3-2*q1,   2*pbar_y*q0-2*q2,   2*pbar_y*q1-2*q3];
    
    F_pbar_q = (1/p_zc) * [M4 * v_r, M5 * v_r]' * dt;  % CHECKED: TRUE
    F(idx_pbar, idx_q) = F_pbar_q;
    
    %% F_pbar_vr: Image feature to velocity
    Lv = [-1/p_zc,     0,    pbar_x/p_zc;   % CHECKED: TRUE
           0,     -1/p_zc,   pbar_y/p_zc];
    F(idx_pbar, idx_vr) = Lv * R_b2c * R_b2e' * dt;  % CHECKED: TRUE
    
    %% F_pbar_pbar: Image feature to image feature
    v_cam = R_b2c * R_b2e' * v_r;
    omega_cam = R_b2c * omega;
    vz_c = v_cam(3);
    ox_c = omega_cam(1); oy_c = omega_cam(2); oz_c = omega_cam(3);
    
    F_pbar_pbar = eye(2) + [vz_c/p_zc + pbar_y*ox_c - 2*pbar_x*oy_c,  pbar_x*ox_c + oz_c;   % CHECKED: TRUE
                            -pbar_y*oy_c - oz_c,  vz_c/p_zc + 2*pbar_y*ox_c - pbar_x*oy_c] * dt;
    F(idx_pbar, idx_pbar) = F_pbar_pbar;
    
    %% F_pbar_bgyr: Image feature to gyro bias
    Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;   %CHECKED: TRUE
          (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
    F(idx_pbar, idx_bgyr) = -Lw * R_b2c * dt;
    
    %% Process noise Jacobian G (18x6)
    G = zeros(18, 6);
    
    % G for quaternion (from gyro bias noise)
    G(1:4, 1:3) = F_q_bgyr;  %CHECKED: TRUE
    
    % G for velocity (from accel bias noise)
    G(idx_vr, 4:6) = -R_b2e * dt;
    
    % G for image feature (from gyro bias noise)
    G(idx_pbar, 1:3) = F(idx_pbar, idx_bgyr);
end

function plot_results(t, x_true, x_est, P_diag, idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc)
    % Plot DKF estimation results
    
    %% ==================== MAIN STATE ESTIMATION FIGURE ====================
    figure('Name', 'DKF State Estimation', 'Position', [100, 100, 1400, 900]);
    
    %% Relative Position - All 3 Components
    subplot(3, 2, 1);
    colors_true = {'b-', 'g-', 'm-'};
    colors_est = {'bo', 'go', 'mo'};
    labels = {'X', 'Y', 'Z'};
    hold on;
    for i = 1:3
        plot(t, x_true(idx_pr(i), :), colors_true{i}, 'LineWidth', 1.5);
    end
    for i = 1:3
        plot(t, x_est(idx_pr(i), :), colors_est{i}, 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Position (m)');
    title('Relative Position (All Components)');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(3, 2, 2);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_pr(i), :) - x_est(idx_pr(i), :), 'LineWidth', 1.5);
    end
    plot(t, vecnorm(x_true(idx_pr, :) - x_est(idx_pr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Position Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Norm', 'Location', 'best');
    grid on;
    
    %% Relative Velocity - All 3 Components
    subplot(3, 2, 3);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_vr(i), :), colors_true{i}, 'LineWidth', 1.5);
    end
    for i = 1:3
        plot(t, x_est(idx_vr(i), :), colors_est{i}, 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    title('Relative Velocity (All Components)');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(3, 2, 4);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_vr(i), :) - x_est(idx_vr(i), :), 'LineWidth', 1.5);
    end
    plot(t, vecnorm(x_true(idx_vr, :) - x_est(idx_vr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m/s)');
    title('Velocity Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Norm', 'Location', 'best');
    grid on;
    
    %% Image Feature - Both Components
    subplot(3, 2, 5);
    hold on;
    plot(t, x_true(idx_pbar(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(1), :), 'bo', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(2), :), 'go', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Normalized Coordinate');
    title('Normalized Image Features');
    legend('True p_x', 'True p_y', 'Est p_x', 'Est p_y', 'Location', 'best');
    grid on;
    
    subplot(3, 2, 6);
    hold on;
    plot(t, x_true(idx_pbar(1), :) - x_est(idx_pbar(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2), :) - x_est(idx_pbar(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, vecnorm(x_true(idx_pbar, :) - x_est(idx_pbar, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error');
    title('Image Feature Estimation Error');
    legend('Error p_x', 'Error p_y', 'Norm', 'Location', 'best');
    grid on;
    
    sgtitle('Delayed Kalman Filter - State Estimation Results');
    
    %% ==================== BIAS ESTIMATION FIGURE ====================
    figure('Name', 'Bias Estimation', 'Position', [150, 150, 1200, 600]);
    
    subplot(2, 2, 1);
    hold on;
    plot(t, x_true(idx_bgyr(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bgyr(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bgyr(3), :), 'm-', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(1), :), 'bo', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(2), :), 'go', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(3), :), 'mo', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Gyro Bias (rad/s)');
    title('Gyroscope Bias Estimation');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 2);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_bgyr(i), :) - x_est(idx_bgyr(i), :), 'LineWidth', 1.5);
    end
    plot(t, vecnorm(x_true(idx_bgyr, :) - x_est(idx_bgyr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (rad/s)');
    title('Gyro Bias Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Norm', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 3);
    hold on;
    plot(t, x_true(idx_bacc(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bacc(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bacc(3), :), 'm-', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(1), :), 'bo', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(2), :), 'go', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(3), :), 'mo', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Accel Bias (m/s^2)');
    title('Accelerometer Bias Estimation');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 4);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_bacc(i), :) - x_est(idx_bacc(i), :), 'LineWidth', 1.5);
    end
    plot(t, vecnorm(x_true(idx_bacc, :) - x_est(idx_bacc, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m/s^2)');
    title('Accel Bias Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Norm', 'Location', 'best');
    grid on;
    
    sgtitle('IMU Bias Estimation Results');
    
    %% ==================== ATTITUDE ESTIMATION FIGURE ====================
    figure('Name', 'Attitude Estimation', 'Position', [200, 200, 1200, 400]);
    
    eul_true = zeros(3, size(x_true, 2));
    eul_est = zeros(3, size(x_est, 2));
    for i = 1:size(x_true, 2)
        eul_true(:, i) = quat2eul(x_true(idx_q, i)', 'ZYX')' * 180/pi;
        eul_est(:, i) = quat2eul(x_est(idx_q, i)', 'ZYX')' * 180/pi;
    end
    
    subplot(1, 3, 1);
    plot(t, eul_true(1, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(1, :), 'ro', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Yaw (deg)');
    title('Yaw Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(1, 3, 2);
    plot(t, eul_true(2, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(2, :), 'ro', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Pitch (deg)');
    title('Pitch Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(1, 3, 3);
    plot(t, eul_true(3, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(3, :), 'ro', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Roll (deg)');
    title('Roll Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    sgtitle('Attitude Estimation Results');
end
