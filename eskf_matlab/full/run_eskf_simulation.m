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
%   Nominal state x (21): [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3), bmag(3)]
%   Error state δx (20): [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3), δbmag(3)]
%
% File Structure:
%   - ErrorStateKalmanFilter.m : ESKF class with prediction/correction methods
%   - compute_eskf_jacobians.m : Error state Jacobian computation
%   - compute_image_features.m : Normalized image coordinate computation
%   - plot_eskf_results.m      : Visualization functions

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));
rng(32)

%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 1/200;           % IMU update rate: 200 Hz
dt_eskf = 1/200;          % ESKF update rate: 100 Hz (can be different from IMU)
dt_image = 1/30;          % Image update rate: ~30 Hz
dt_radar = 1/0.5;         % RADAR update rate: 0.5 Hz
dt_mag = 1/20;            % Magnetometer update rate: 20 Hz
t_delay = 0/1000;        % Image processing delay: 80 ms
t_total = 25;             % Total simulation time
D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Flag for measurement devices
useRadar = true;
useCam   = true;
useMag   = true;

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
imu_params.fs = 1/dt_imu;          % Sample rate [Hz]
imu_params.sigma_a_n     = 0.19953898572;            %matlab-ver 0.17121490151;       % Accel noise [m/s²]
imu_params.sigma_omega_n = 0.00340002349;            %matlab-ver 0.00831816814;       % Gyro noise [rad/s]
imu_params.sigma_a_w     = 5.5379948939679005e-04;   %matlab-ver 4e-05;               % Accel bias RW [m/s²√s]
imu_params.sigma_omega_w = 0.00001557916;            %matlab-ver 2e-06;               % Gyro bias RW [rad/s√s]
imu_params.omega_b_init  = [0.005; -0.003; 0.002];    % Initial gyro bias [rad/s]
imu_params.a_b_init      = [0.02; -0.01; 0.015];          % Initial accel bias [m/s²]

% Initial bias values
imu_params.omega_b_init = [0.005; -0.003; 0.002];  % ω_b(0) [rad/s]
imu_params.a_b_init = [0.02; -0.01; 0.015];        % a_b(0) [m/s²]

% Create IMU model
imu = IMUModel(imu_params);

% Get process noise covariance for ESKF (matches Gc structure)
Qi_imu = imu.getDiscreteProcessNoise(dt_eskf);  % 12x12
Qc_imu = imu.getESKFProcessNoise();

%% ======================== MAGNETOMETER MODEL ========================
% Magnetometer configuration (20 Hz update rate)
mag_params = struct();
mag_params.fs = 1/dt_mag;              % 20 Hz
mag_params.sigma_mag_n = 0.01;         % Magnetometer noise std
mag_params.sigma_mag_w = 1e-5;         % Magnetometer bias random walk
mag_params.bias_mag_init = [0.001; -0.002; 0.001];  % Initial mag bias

% Create magnetometer model
mag = MagnetometerModel(mag_params);

% Reference magnetic field in NED frame (normalized)
% Typical for mid-latitudes (e.g., Turkey)
B_ned = [0.56; 0.04; 0.80];

% Get magnetometer process noise
Mi = mag.getDiscreteProcessNoise(dt_eskf);  % 3x3
sigma_mag_w_sq = mag.getContinuousProcessNoise();

% Extend process noise matrices to include magnetometer bias
% Qi: 12x12 -> 15x15, Qc: 12x12 -> 15x15
Qi = blkdiag(Qi_imu, Mi);
Qc = blkdiag(Qc_imu, sigma_mag_w_sq * eye(3));

%% ======================== SENSOR NOISE PARAMETERS ========================
% Image sensor noise
sigma_img = 0.05;        % Normalized image coordinate noise (~5 pixels)

% ---- Realistic RADAR model (spherical noise domain) ----
% Instantiate RadarEmulator with per-axis RMSE values.
% The emulator injects noise in the radar's native spherical coordinates
% (range, azimuth, elevation, Doppler) — not in Cartesian space — which
% correctly reproduces the cone-shaped error volume of real radar hardware.
radar_params = struct();
radar_params.rmse_range     = 3.0;           % Range RMSE             [m]
radar_params.rmse_azimuth   = deg2rad(1.3);  % Azimuth RMSE           [rad]   (~1.3 deg)
radar_params.rmse_elevation = deg2rad(3.3);  % Elevation RMSE         [rad]   (~3.3 deg)
radar_params.rmse_doppler   = 0.5;           % Radial velocity RMSE   [m/s]
% Geometry and timing stored as class properties (constant for the whole flight)
radar_params.ref_lla0  = [];    % filled after ref_lla0 is computed below
radar_params.radar_lla = [];    % filled after radar_lla is computed below
radar_params.dt        = dt_radar;           % Measurement period [s]

% Measurement noise covariance for ESKF correctRadar:
% Computed dynamically at each radar update from the estimated target
% geometry (range, azimuth, elevation) using the spherical Jacobian.
% An initial isotropic fallback is provided here and will be overwritten
% at the first radar update.
R_radar_init = diag([radar_params.rmse_range^2 * ones(1,3), ...
                     radar_params.rmse_doppler^2 * ones(1,3)]);

% Measurement noise covariance - Image (2x2)
R_img = sigma_img^2 * eye(2);

% Measurement noise covariance - Magnetometer (3x3)
sigma_mag = 0.01;  % Magnetometer measurement noise
R_mag = sigma_mag^2 * eye(3);

%% ======================== INITIAL CONDITIONS ========================
% ===== GEODETIC REFERENCE ORIGIN (for RadarEmulator) =====
% The ESKF works in a local Cartesian NED frame. We define a geodetic
% anchor point from which all LLA coordinates are derived.  Both the
% radar and the ESKF reference frame share this origin.
ref_lla0     = [39.9; 32.9; 0];     % [lat, lon, alt]  NED origin (deg, deg, m)
radar_offset = [0; 0; 0];           % Radar NED offset from ref_lla0 [m] (co-located here)

% Convert radar NED offset to LLA
[ecef_rx, ecef_ry, ecef_rz] = ned2ecef( ...
    radar_offset(1), radar_offset(2), radar_offset(3), ...
    ref_lla0(1), ref_lla0(2), ref_lla0(3), referenceEllipsoid('wgs84'));
[r_lat, r_lon, r_alt] = ecef2geodetic( ...
    referenceEllipsoid('wgs84'), ecef_rx, ecef_ry, ecef_rz);
radar_lla = [r_lat; r_lon; r_alt];  % Radar geodetic location

% Now that ref_lla0 and radar_lla are known, finalise and build the emulator
radar_params.ref_lla0  = ref_lla0;
radar_params.radar_lla = radar_lla;
radar_emulator = RadarEmulator(radar_params);

% ===== INTERCEPTOR INITIAL STATE =====
p_int = [0; 0; -0];          % Interceptor position (NED)
v_int = [0; 0; 0];            % Interceptor velocity

% Attitude (level, pointing toward target)
yaw_init = 0;
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

% ===== TARGET INITIAL STATE =====
p_tgt = [80; 30; -40];        % Target position
v_tgt = [0; 0; 0];            % Target velocity

% ===== RELATIVE STATE =====
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

% ===== IMU BIASES (from IMU model, paper notation) =====
% Biases evolve over time according to random walk model
omega_b_true = imu.omega_b;    % ω_b: Gyro bias from IMU model [rad/s]
a_b_true = imu.a_b;            % a_b: Accel bias from IMU model [m/s²]

%% ======================== STATE INDICES (NOMINAL STATE 21-dim) ========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;
idx_bmag = 19:21;

%% ======================== ERROR STATE INDICES (20-dim) ========================
idx_dtheta = 1:3;
idx_dpr = 4:6;
idx_dvr = 7:9;
idx_dpbar = 10:11;
idx_dbgyr = 12:14;
idx_dbacc = 15:17;
idx_dbmag = 18:20;

%% ======================== COMPUTE INITIAL IMAGE FEATURES ========================
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);

%% ======================== INITIALIZE TRUE STATE ========================
x_true = [q_true; p_r_true; v_r_true; pbar_true; omega_b_true; a_b_true; mag.bias_mag];

%% ======================== INITIALIZE ESKF ========================
% Define initial errors (intuitive Euler angles in degrees)
init_errors = struct();
init_errors.euler_deg  = [1; 1; 1];       % [yaw; pitch; roll] errors in degrees
init_errors.position   = [0.5; 0.5; 0.5]; % Position error [m]
init_errors.velocity   = [0.1; -0.05; 0]; % Velocity error [m/s]
init_errors.pbar       = [0.01; -0.02];   % Image feature error
init_errors.b_gyr      = [0 ; 0 ; 0];     % Start with zero bias estimate
init_errors.b_acc      = [0 ; 0 ; 0];     % Start with zero bias estimate
init_errors.b_mag      = [0 ; 0 ; 0];     % Start with zero mag bias estimate

% Create initial nominal state using ESKF static method
x_init = ErrorStateKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                    pbar_true, omega_b_true, a_b_true, ...
                                                    init_errors);

% Define initial covariance standard deviations for ERROR STATE (17x17)
init_sigma = struct();
init_sigma.attitude   = 0.05;     % ~3° attitude uncertainty (radians for δθ)
init_sigma.position   = 3;        % 3m position uncertainty
init_sigma.velocity   = 0.5;      % 0.5 m/s velocity uncertainty
init_sigma.pbar       = 0.01;      % Image feature uncertainty
init_sigma.b_gyr      = 0.005;    % Gyro bias uncertainty (rad/s)
init_sigma.b_acc      = 0.05;     % Accel bias uncertainty (m/s²)
init_sigma.b_mag      = 0.005;    % Mag bias uncertainty

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
eskf_params.R_radar = R_radar_init;
eskf_params.R_mag = R_mag;
eskf_params.B_ned = B_ned;
eskf_params.x_init = x_init;      % 21x1 nominal state
eskf_params.P_init = P_init;      % 20x20 error covariance
eskf_params.delay_steps = D;

eskf = ErrorStateKalmanFilter(eskf_params);

%% ======================== STORAGE FOR RESULTS ========================
x_true_log = zeros(21, N_imu);    % True nominal state
x_est_log = zeros(21, N_imu);     % Estimated nominal state
P_log = zeros(20, N_imu);         % Error covariance diagonal (20-dim)
omega_b_log = zeros(3, N_imu);    % True gyro bias ω_b (evolving)
a_b_log = zeros(3, N_imu);        % True accel bias a_b (evolving)
bias_mag_log = zeros(3, N_imu);   % True mag bias (evolving)

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('Starting ESKF Simulation...\n');
fprintf('Total time: %.1f s, IMU rate: %.0f Hz, ESKF rate: %.0f Hz, Image rate: %.0f Hz, RADAR rate: %.1f Hz, Mag rate: %.0f Hz\n', ...
        t_total, 1/dt_imu, 1/dt_eskf, 1/dt_image, 1/dt_radar, 1/dt_mag);
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

mag_update_counter = 0;
mag_sample_idx = round(dt_mag / dt_imu);

for k = 1:N_imu
    t = t_imu(k);
    
    %% ==================== TRUE DYNAMICS ====================
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, idx_bmag);
    
    % Generate IMU measurements using realistic IMU model
    % (includes evolving biases via random walk and proper noise)
    [omega_meas, a_meas, omega_b_true, a_b_true] = imu.measure(omega_true, a_body_true);
    
    % Update true state with current biases (for accurate error computation)
    x_true(idx_bgyr) = omega_b_true;
    x_true(idx_bacc) = a_b_true;
    % x_true(idx_bmag) = x_true(idx_bmag);
    
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

            % ---- Realistic radar measurement via RadarEmulator ----
            %
            % The target's absolute NED position (relative to ref_lla0):
            %   p_tgt_ned0 = p_int - p_r_true   (since p_r = p_int - p_tgt)
            p_int_current   = p_int;                      % Interceptor NED position   [m]
            p_tgt_ned0      = p_int_current - x_true(idx_pr);  % Target NED (ref_lla0)  [m]
            v_tgt_ned_true  = v_tgt;                      % Target velocity NED        [m/s]

            % Convert target NED position to geodetic LLA
            [ecef_tx, ecef_ty, ecef_tz] = ned2ecef( ...
                p_tgt_ned0(1), p_tgt_ned0(2), p_tgt_ned0(3), ...
                ref_lla0(1), ref_lla0(2), ref_lla0(3), referenceEllipsoid('wgs84'));
            [t_lat, t_lon, t_alt] = ecef2geodetic( ...
                referenceEllipsoid('wgs84'), ecef_tx, ecef_ty, ecef_tz);
            target_lla = [t_lat; t_lon; t_alt];

            % ---- Dynamic anisotropic R_radar from estimated target geometry ----
            [z_pos_ned0, z_vel_ned0, R_pos, R_vel] = ...
                radar_emulator.emulate_measurement(target_lla, v_tgt_ned_true);
            
            eskf.R_radar = blkdiag(R_pos, R_vel);       % update before correction

            % Build radar measurement vector [pos; vel] relative to interceptor
            % The ESKF correctRadar expects [p_r; v_r] = [p_int - p_tgt; v_int - v_tgt]
            p_int_ned0 = p_int_current;        % re-use for clarity
            z_radar = [(p_int_ned0 - z_pos_ned0); ...  % measured relative position [m]
                       (v_int     - z_vel_ned0)];       % measured relative velocity [m/s]

            % Apply RADAR correction
            eskf.correctRadar(z_radar);
        end
    end
    
    %% ==================== MAGNETOMETER CORRECTION (20 Hz) ====================
    mag_update_counter = mag_update_counter + 1;
    if mag_update_counter >= mag_sample_idx && useMag
        mag_update_counter = 0;
        
        % Get true rotation for magnetometer measurement
        R_b2e_true = quat2rotm(x_true(idx_q)');        

        % Generate magnetometer measurement (with evolving bias)
        [mag_meas, bias_mag_true] = mag.measure(R_b2e_true, B_ned);
       
        x_true(idx_bmag) = bias_mag_true;

        % Apply magnetometer correction
        eskf.correctMag(mag_meas);
    end
    
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = eskf.x;
    P_log(:, k) = diag(eskf.P);
    omega_b_log(:, k) = omega_b_true;  % Log evolving gyro bias ω_b
    a_b_log(:, k) = a_b_true;          % Log evolving accel bias a_b
    bias_mag_log(:, k) = x_true(idx_bmag);  % Log evolving mag bias
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%%\n', 100*k/N_imu);
    end
end

fprintf('\nSimulation Complete!\n');

%% ======================== PLOT RESULTS ========================
plot_eskf_results(t_imu, x_true_log, x_est_log, P_log, ...
                  idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, idx_bmag, ...
                  idx_dtheta, idx_dpr, idx_dvr, idx_dpbar, idx_dbgyr, idx_dbacc, idx_dbmag);

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

% Mag bias error
bmag_error = x_true_log(idx_bmag, :) - x_est_log(idx_bmag, :);
bmag_rmse = sqrt(mean(vecnorm(bmag_error, 2, 1).^2));
fprintf('Mag Bias RMSE: %.6f\n', bmag_rmse);

fprintf('\n=== Final Error Statistics ===\n');
fprintf('Final Position Error: [%.3f, %.3f, %.3f] m\n', pos_error(:,end)');
fprintf('Final Velocity Error: [%.3f, %.3f, %.3f] m/s\n', vel_error(:,end)');
fprintf('Final Attitude Error: %.3f deg\n', att_error_deg(end));
