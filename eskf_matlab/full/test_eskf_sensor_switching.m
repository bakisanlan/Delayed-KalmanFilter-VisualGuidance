%% ESKF Sensor Switching Test - Camera Fault Scenario
% This script tests ESKF behavior under the following scenario:
%   Phase 1 (0-10s):   Radar ONLY (camera OFF)
%   Phase 2 (10-20s):  Radar + Camera with CONSTANT FALSE measurement
%   Phase 3 (20-40s):  Radar ONLY (camera OFF again)
%
% Purpose: Observe how the filter handles camera sensor failure and recovery

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));
rng(42)  % Different seed for reproducibility

%% ======================== SIMULATION PARAMETERS ========================
% Time settings
dt_imu = 1/200;           % IMU update rate: 200 Hz
dt_eskf = 1/200;          % ESKF update rate: 200 Hz
dt_image = 1/30;          % Image update rate: ~30 Hz
dt_radar = 1/0.5;         % RADAR update rate: 0.5 Hz
t_delay = 0/1000;         % Image processing delay: 0 ms
t_total = 140;             % Total simulation time

D = round(t_delay / dt_imu);  % Delay in IMU cycles

% Time phases for sensor switching
t_phase1_end = 1;     % End of radar-only phase
t_phase2_end = 100;   % End of camera + radar phase (camera with false measurement)
% Phase 3: t_phase2_end to t_total (radar only again)

% Time vectors
t_imu = 0:dt_imu:t_total;
N_imu = length(t_imu);

%% ======================== PHYSICAL CONSTANTS ========================
g = 9.81;                 % Gravity (m/s^2)
e3 = [0; 0; 1];           % Unit vector in z-direction

%% ======================== CAMERA PARAMETERS ========================
R_c2b = [0 0 1;
         1 0 0; 
         0 1 0];          % Transforms vector FROM camera TO body frame
R_b2c = R_c2b';           % Transforms vector FROM body TO camera frame

%% ======================== IMU MODEL ========================
imu_params = struct();
imu_params.fs = 1/dt_imu;          % Sample rate [Hz]
imu_params.sigma_a_n     = 0.17121490151;        % Accel noise [m/s²]
imu_params.sigma_omega_n = 0.00831816814;       % Gyro noise [rad/s]
imu_params.sigma_a_w     = 4e-05;                    % Accel bias RW [m/s²√s]
imu_params.sigma_omega_w = 2e-06;                    % Gyro bias RW [rad/s√s]
imu_params.omega_b_init = [0.005; -0.003; 0.002];    % Initial gyro bias [rad/s]
imu_params.a_b_init = [0.02; -0.01; 0.015];          % Initial accel bias [m/s²]

imu = IMUModel(imu_params);

Qi = imu.getDiscreteProcessNoise(dt_eskf);
Qc = imu.getESKFProcessNoise();

%% ======================== SENSOR NOISE PARAMETERS ========================
sigma_img = 0.05;            % Image coordinate noise
sigma_radar_pos = 1;         % RADAR position noise [m]
sigma_radar_vel = 0.5;       % RADAR velocity noise [m/s]

R_img = sigma_img^2 * eye(2);
R_radar = blkdiag(sigma_radar_pos^2 * eye(3), sigma_radar_vel^2 * eye(3));

%% ======================== CONSTANT FALSE CAMERA MEASUREMENT ========================
% This will be the constant false measurement fed during Phase 2
% Far from the true values to simulate a stuck/faulty camera
pbar_false = [0.3; 0.3];  % Constant false measurement

%% ======================== INITIAL CONDITIONS ========================
p_int = [0; 0; -65];          % Interceptor position (NED)
v_int = [0; 0; 0];            % Interceptor velocity

yaw_init = 0;
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

p_tgt = [1000; 1000; -65];        % Target position
v_tgt = [0; 0; 0];            % Target velocity

p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

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

%% ======================== INITIALIZE ESKF (with slight errors) ========================
% Initialize with SLIGHT errors (correct initialization with small error)
init_errors = struct();
init_errors.euler_deg  = [0.5; 0.5; 0.5];     % Small attitude errors [deg]
init_errors.position   = [0.2; 0.2; 0.2];     % Small position error [m]
init_errors.velocity   = [0.05; -0.05; 0];    % Small velocity error [m/s]
init_errors.pbar       = [0.005; -0.005];     % Small image feature error
init_errors.b_gyr      = [0; 0; 0];           % Start with zero bias estimate
init_errors.b_acc      = [0; 0; 0];           % Start with zero bias estimate

x_init = ErrorStateKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                    pbar_true, omega_b_true, a_b_true, ...
                                                    init_errors);

init_sigma = struct();
init_sigma.attitude   = 0.02;     % ~1° attitude uncertainty
init_sigma.position   = 1;        % 1m position uncertainty
init_sigma.velocity   = 0.2;      % 0.2 m/s velocity uncertainty
init_sigma.pbar       = 0.01;     % Image feature uncertainty
init_sigma.b_gyr      = 0.005;    % Gyro bias uncertainty
init_sigma.b_acc      = 0.05;     % Accel bias uncertainty

P_init = ErrorStateKalmanFilter.createInitialCovariance(init_sigma);

eskf_params = struct();
eskf_params.dt_imu = dt_imu;
eskf_params.dt_eskf = dt_eskf;
eskf_params.R_b2c = R_b2c;
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
omega_b_log = zeros(3, N_imu);
a_b_log = zeros(3, N_imu);

% Additional logging for sensor activity
sensor_status_log = zeros(2, N_imu);  % [radar_active; camera_active]
camera_meas_log = zeros(2, N_imu);    % Camera measurements used
pbar_true_log = zeros(2, N_imu);      % True pbar values

%% ======================== MAIN SIMULATION LOOP ========================
fprintf('=============================================================\n');
fprintf('ESKF Sensor Switching Test - Camera Fault Scenario\n');
fprintf('=============================================================\n');
fprintf('Phase 1 (0-%.0fs):   Radar ONLY\n', t_phase1_end);
fprintf('Phase 2 (%.0f-%.0fs): Radar + Camera (CONSTANT FALSE measurement)\n', t_phase1_end, t_phase2_end);
fprintf('Phase 3 (%.0f-%.0fs): Radar ONLY (recovery)\n', t_phase2_end, t_total);
fprintf('=============================================================\n\n');

fprintf('Starting simulation...\n');

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
    
    %% ==================== DETERMINE SENSOR AVAILABILITY ====================
    % Phase 1: Radar only (0 to t_phase1_end)
    % Phase 2: Radar + Camera with false measurement (t_phase1_end to t_phase2_end)
    % Phase 3: Radar only (t_phase2_end to t_total)
    
    if t < t_phase1_end
        useRadar = true;
        useCam = false;
        useFalseCameraMeas = false;
    elseif t < t_phase2_end
        useRadar = true;
        useCam = true;
        useFalseCameraMeas = true;  % Use constant false measurement
    else
        useRadar = true;
        useCam = false;
        useFalseCameraMeas = false;
    end
    
    sensor_status_log(:, k) = [useRadar; useCam];
    
    %% ==================== TRUE DYNAMICS ====================
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Generate IMU measurements
    [omega_meas, a_meas, omega_b_true, a_b_true] = imu.measure(omega_true, a_body_true);
    
    % Update true state with current biases
    x_true(idx_bgyr) = omega_b_true;
    x_true(idx_bacc) = a_b_true;
    
    % Log true pbar
    pbar_true_log(:, k) = x_true(idx_pbar);
    
    %% ==================== ESKF PREDICTION ====================
    omega_accum = omega_accum + omega_meas;
    a_accum = a_accum + a_meas;
    imu_count = imu_count + 1;
    
    eskf_update_counter = eskf_update_counter + 1;
    
    if eskf_update_counter >= eskf_sample_idx
        eskf_update_counter = 0;
        
        omega_avg = omega_accum / imu_count;
        a_avg = a_accum / imu_count;
        
        eskf.predict(omega_avg, a_avg, t);
        
        omega_accum = zeros(3, 1);
        a_accum = zeros(3, 1);
        imu_count = 0;
    end
    
    %% ==================== CAMERA CORRECTION ====================
    image_update_counter = image_update_counter + 1;
    
    if image_update_counter >= image_sample_idx && k > D
        image_update_counter = 0;
        
        if useCam
            if useFalseCameraMeas
                % Use CONSTANT FALSE measurement
                z_meas = pbar_false;
                camera_meas_log(:, k) = z_meas;
                eskf.correctImage(z_meas, D);
            else
                % Normal camera measurement (not used in this test)
                R_b2e_delayed = quat2rotm(x_true(idx_q)');
                p_r_cam_delayed = R_b2c * R_b2e_delayed' * (-x_true(idx_pr));
                
                if p_r_cam_delayed(3) > 2
                    z_meas = [p_r_cam_delayed(1)/p_r_cam_delayed(3); 
                             p_r_cam_delayed(2)/p_r_cam_delayed(3)] + sigma_img * randn(2,1);
                    camera_meas_log(:, k) = z_meas;
                    eskf.correctImage(z_meas, D);
                end
            end
        end
    end
    
    %% ==================== RADAR CORRECTION ====================
    radar_update_counter = radar_update_counter + 1;
    if useRadar
        if radar_update_counter >= radar_sample_idx
            radar_update_counter = 0;
            
            p_r_true_current = x_true(idx_pr);
            v_r_true_current = x_true(idx_vr);
            z_radar = [p_r_true_current + sigma_radar_pos * randn(3,1);
                       v_r_true_current + sigma_radar_vel * randn(3,1)];
            
            eskf.correctRadar(z_radar);
        end
    end
    
    %% ==================== STORE RESULTS ====================
    x_true_log(:, k) = x_true;
    x_est_log(:, k) = eskf.x;
    P_log(:, k) = diag(eskf.P);
    omega_b_log(:, k) = omega_b_true;
    a_b_log(:, k) = a_b_true;
    
    %% ==================== PHASE TRANSITION MESSAGES ====================
    if k > 1
        prev_t = t_imu(k-1);
        if prev_t < t_phase1_end && t >= t_phase1_end
            fprintf('\n>>> Phase 2 Started: Camera with FALSE measurement activated at t=%.1fs\n', t);
        elseif prev_t < t_phase2_end && t >= t_phase2_end
            fprintf('\n>>> Phase 3 Started: Camera DISABLED, Radar only at t=%.1fs\n', t);
        end
    end
    
    %% ==================== PROGRESS DISPLAY ====================
    if mod(k, round(N_imu/10)) == 0
        fprintf('Progress: %.0f%% (t=%.1fs)\n', 100*k/N_imu, t);
    end
end

fprintf('\n=============================================================\n');
fprintf('Simulation Complete!\n');
fprintf('=============================================================\n\n');

%% ======================== PLOT RESULTS ========================
% Use the standard plotting function
plot_eskf_results(t_imu, x_true_log, x_est_log, P_log, ...
                  idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, ...
                  idx_dtheta, idx_dpr, idx_dvr, idx_dpbar, idx_dbgyr, idx_dbacc);

%% ======================== ADDITIONAL PLOTS FOR THIS TEST ========================
% Plot sensor status and errors with phase markers

figure('Name', 'Sensor Switching Analysis', 'Position', [100, 100, 1400, 900]);

% --- Subplot 1: Position Error ---
subplot(3, 2, 1);
pos_error = x_true_log(idx_pr, :) - x_est_log(idx_pr, :);
pos_error_norm = vecnorm(pos_error, 2, 1);
plot(t_imu, pos_error_norm, 'b', 'LineWidth', 1.5);
hold on;
xline(t_phase1_end, 'r--', 'Phase 2: False Cam', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'Phase 3: Radar Only', 'LineWidth', 2);
ylabel('Position Error [m]');
xlabel('Time [s]');
title('Position Error Norm');
grid on;
legend('Error', 'Location', 'best');

% --- Subplot 2: Velocity Error ---
subplot(3, 2, 2);
vel_error = x_true_log(idx_vr, :) - x_est_log(idx_vr, :);
vel_error_norm = vecnorm(vel_error, 2, 1);
plot(t_imu, vel_error_norm, 'b', 'LineWidth', 1.5);
hold on;
xline(t_phase1_end, 'r--', 'Phase 2: False Cam', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'Phase 3: Radar Only', 'LineWidth', 2);
ylabel('Velocity Error [m/s]');
xlabel('Time [s]');
title('Velocity Error Norm');
grid on;

% --- Subplot 3: pbar (Image Feature) Error ---
subplot(3, 2, 3);
pbar_error = x_true_log(idx_pbar, :) - x_est_log(idx_pbar, :);
pbar_error_norm = vecnorm(pbar_error, 2, 1);
plot(t_imu, pbar_error_norm, 'b', 'LineWidth', 1.5);
hold on;
xline(t_phase1_end, 'r--', 'Phase 2: False Cam', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'Phase 3: Radar Only', 'LineWidth', 2);
ylabel('pbar Error');
xlabel('Time [s]');
title('Image Feature Error Norm');
grid on;

% --- Subplot 4: pbar True vs Estimated vs False Measurement ---
subplot(3, 2, 4);
plot(t_imu, pbar_true_log(1, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'True pbar_x');
hold on;
plot(t_imu, x_est_log(idx_pbar(1), :), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Est pbar_x');
yline(pbar_false(1), 'k:', 'False meas', 'LineWidth', 2);
xline(t_phase1_end, 'r--', 'LineWidth', 1);
xline(t_phase2_end, 'g--', 'LineWidth', 1);
ylabel('pbar_x');
xlabel('Time [s]');
title('Image Feature X: True vs Estimated vs False');
legend('Location', 'best');
grid on;

% --- Subplot 5: Attitude Error ---
subplot(3, 2, 5);
att_error_deg = zeros(1, N_imu);
for k = 1:N_imu
    q_true_k = x_true_log(idx_q, k);
    q_est_k = x_est_log(idx_q, k);
    q_err = quatmultiply(q_true_k', quatconj(q_est_k'))';
    att_error_deg(k) = 2 * acosd(min(abs(q_err(1)), 1));
end
plot(t_imu, att_error_deg, 'b', 'LineWidth', 1.5);
hold on;
xline(t_phase1_end, 'r--', 'Phase 2: False Cam', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'Phase 3: Radar Only', 'LineWidth', 2);
ylabel('Attitude Error [deg]');
xlabel('Time [s]');
title('Attitude Error');
grid on;

% --- Subplot 6: Sensor Status Timeline ---
subplot(3, 2, 6);
area(t_imu, sensor_status_log(1, :) * 1, 'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.5, 'DisplayName', 'Radar');
hold on;
area(t_imu, sensor_status_log(2, :) * 0.5, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5, 'DisplayName', 'Camera (False Meas)');
xline(t_phase1_end, 'r--', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'LineWidth', 2);
ylabel('Sensor Active');
xlabel('Time [s]');
title('Sensor Activation Timeline');
legend('Location', 'best');
yticks([0 0.5 1]);
yticklabels({'Off', 'Cam', 'Radar'});
grid on;

sgtitle('ESKF Sensor Switching Test: Camera Fault Scenario', 'FontSize', 14, 'FontWeight', 'bold');

%% ======================== COVARIANCE ANALYSIS ========================
figure('Name', 'Covariance Analysis', 'Position', [150, 150, 1200, 600]);

% Position covariance
subplot(2, 2, 1);
plot(t_imu, sqrt(P_log(idx_dpr(1), :)), 'r', 'LineWidth', 1.5, 'DisplayName', 'σ_{px}');
hold on;
plot(t_imu, sqrt(P_log(idx_dpr(2), :)), 'g', 'LineWidth', 1.5, 'DisplayName', 'σ_{py}');
plot(t_imu, sqrt(P_log(idx_dpr(3), :)), 'b', 'LineWidth', 1.5, 'DisplayName', 'σ_{pz}');
xline(t_phase1_end, 'r--', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Position Std [m]');
title('Position Uncertainty (1σ)');
legend('Location', 'best');
grid on;

% Velocity covariance
subplot(2, 2, 2);
plot(t_imu, sqrt(P_log(idx_dvr(1), :)), 'r', 'LineWidth', 1.5, 'DisplayName', 'σ_{vx}');
hold on;
plot(t_imu, sqrt(P_log(idx_dvr(2), :)), 'g', 'LineWidth', 1.5, 'DisplayName', 'σ_{vy}');
plot(t_imu, sqrt(P_log(idx_dvr(3), :)), 'b', 'LineWidth', 1.5, 'DisplayName', 'σ_{vz}');
xline(t_phase1_end, 'r--', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Velocity Std [m/s]');
title('Velocity Uncertainty (1σ)');
legend('Location', 'best');
grid on;

% pbar covariance
subplot(2, 2, 3);
plot(t_imu, sqrt(P_log(idx_dpbar(1), :)), 'r', 'LineWidth', 1.5, 'DisplayName', 'σ_{pbar_x}');
hold on;
plot(t_imu, sqrt(P_log(idx_dpbar(2), :)), 'g', 'LineWidth', 1.5, 'DisplayName', 'σ_{pbar_y}');
xline(t_phase1_end, 'r--', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('pbar Std');
title('Image Feature Uncertainty (1σ)');
legend('Location', 'best');
grid on;

% Attitude covariance
subplot(2, 2, 4);
att_cov_deg = rad2deg(sqrt(P_log(idx_dtheta, :)));
plot(t_imu, att_cov_deg(1, :), 'r', 'LineWidth', 1.5, 'DisplayName', 'σ_{θx}');
hold on;
plot(t_imu, att_cov_deg(2, :), 'g', 'LineWidth', 1.5, 'DisplayName', 'σ_{θy}');
plot(t_imu, att_cov_deg(3, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'σ_{θz}');
xline(t_phase1_end, 'r--', 'LineWidth', 2);
xline(t_phase2_end, 'g--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Attitude Std [deg]');
title('Attitude Uncertainty (1σ)');
legend('Location', 'best');
grid on;

sgtitle('Covariance Evolution During Sensor Switching', 'FontSize', 14, 'FontWeight', 'bold');

%% ======================== STATISTICS ========================
fprintf('\n=== PHASE-BY-PHASE STATISTICS ===\n\n');

% Phase indices
phase1_idx = t_imu < t_phase1_end;
phase2_idx = (t_imu >= t_phase1_end) & (t_imu < t_phase2_end);
phase3_idx = t_imu >= t_phase2_end;

% Phase 1 statistics
fprintf('--- Phase 1 (0-%.0fs): Radar ONLY ---\n', t_phase1_end);
pos_rmse_p1 = sqrt(mean(pos_error_norm(phase1_idx).^2));
vel_rmse_p1 = sqrt(mean(vel_error_norm(phase1_idx).^2));
att_rmse_p1 = sqrt(mean(att_error_deg(phase1_idx).^2));
fprintf('  Position RMSE: %.3f m\n', pos_rmse_p1);
fprintf('  Velocity RMSE: %.3f m/s\n', vel_rmse_p1);
fprintf('  Attitude RMSE: %.3f deg\n\n', att_rmse_p1);

% Phase 2 statistics
fprintf('--- Phase 2 (%.0f-%.0fs): Radar + FALSE Camera ---\n', t_phase1_end, t_phase2_end);
pos_rmse_p2 = sqrt(mean(pos_error_norm(phase2_idx).^2));
vel_rmse_p2 = sqrt(mean(vel_error_norm(phase2_idx).^2));
att_rmse_p2 = sqrt(mean(att_error_deg(phase2_idx).^2));
pbar_rmse_p2 = sqrt(mean(pbar_error_norm(phase2_idx).^2));
fprintf('  Position RMSE: %.3f m\n', pos_rmse_p2);
fprintf('  Velocity RMSE: %.3f m/s\n', vel_rmse_p2);
fprintf('  Attitude RMSE: %.3f deg\n', att_rmse_p2);
fprintf('  pbar RMSE:     %.4f\n', pbar_rmse_p2);
fprintf('  (False camera measurement used: [%.2f, %.2f])\n\n', pbar_false(1), pbar_false(2));

% Phase 3 statistics
fprintf('--- Phase 3 (%.0f-%.0fs): Radar ONLY (Recovery) ---\n', t_phase2_end, t_total);
pos_rmse_p3 = sqrt(mean(pos_error_norm(phase3_idx).^2));
vel_rmse_p3 = sqrt(mean(vel_error_norm(phase3_idx).^2));
att_rmse_p3 = sqrt(mean(att_error_deg(phase3_idx).^2));
fprintf('  Position RMSE: %.3f m\n', pos_rmse_p3);
fprintf('  Velocity RMSE: %.3f m/s\n', vel_rmse_p3);
fprintf('  Attitude RMSE: %.3f deg\n\n', att_rmse_p3);

% Overall statistics
fprintf('=== OVERALL STATISTICS ===\n');
pos_rmse = sqrt(mean(pos_error_norm.^2));
vel_rmse = sqrt(mean(vel_error_norm.^2));
att_rmse = sqrt(mean(att_error_deg.^2));
fprintf('Position RMSE: %.3f m\n', pos_rmse);
fprintf('Velocity RMSE: %.3f m/s\n', vel_rmse);
fprintf('Attitude RMSE: %.3f deg\n', att_rmse);

fprintf('\n=== ANALYSIS SUMMARY ===\n');
fprintf('The filter behavior during false camera measurements shows:\n');
if pos_rmse_p2 > pos_rmse_p1 * 1.5
    fprintf('  - Significant position degradation during Phase 2\n');
else
    fprintf('  - Position estimation relatively robust to false camera\n');
end
if pos_rmse_p3 < pos_rmse_p2
    fprintf('  - Recovery observed after camera shutdown (Phase 3)\n');
else
    fprintf('  - No significant recovery after camera shutdown\n');
end
