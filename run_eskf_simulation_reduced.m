%% Reduced Error-State Kalman Filter Simulation
% Estimates only target position, target velocity, and image features.
% The interceptor pose and velocity are treated as known inputs.

clear; clc; close all;
rng(32)

%% ======================== SIMULATION PARAMETERS ========================
dt_imu = 1/200;           % IMU / truth propagation rate [Hz]
dt_eskf = 1/200;          % Reduced filter prediction rate [Hz]
dt_image = 1/30;          % Image update rate [Hz]
dt_radar = 1/0.5;         % RADAR update rate [Hz]
t_delay = 0/1000;         % Image processing delay [s]
t_total = 80;             % Total simulation time [s]
D = round(t_delay / dt_imu);

useRadar = true;
useCam = true;

t_imu = 0:dt_imu:t_total;
N_imu = length(t_imu);

%% ======================== PHYSICAL CONSTANTS ===========================
g = 9.81;
e3 = [0; 0; 1];

%% ======================== CAMERA PARAMETERS ============================
R_c2b = [0 0 1;
         1 0 0;
         0 1 0];
R_b2c = R_c2b';

%% ======================== SENSOR PARAMETERS ============================
sigma_img = 0.05;
R_img = sigma_img^2 * eye(2);

sigma_tgt_rw = 1.5;  % Target acceleration random walk intensity [m/s^2]
Qc_reduced = sigma_tgt_rw^2 * eye(3);
Qd_reduced = Qc_reduced * dt_eskf;

radar_params = struct();
radar_params.rmse_range     = 3.0;
radar_params.rmse_azimuth   = deg2rad(1.3);
radar_params.rmse_elevation = deg2rad(3.3);
radar_params.rmse_doppler   = 0.5;
radar_params.dt             = dt_radar;

R_radar_init = diag([radar_params.rmse_range^2 * ones(1, 3), ...
                     radar_params.rmse_doppler^2 * ones(1, 3)]);

%% ======================== INITIAL CONDITIONS ===========================
% Interceptor truth
p_int = [0; 0; 0];
v_int = [0; 1; -1];
yaw_init = 0;
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

% Target truth
p_tgt = [160; 200; -50];
radius = 70;
p_tgt_center = p_tgt - [radius; 0; 0];
v_tgt = [0; 0; 0];

% Full truth state is still used by propagate_true_state
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);
x_true_full = [q_true; p_r_true; v_r_true; pbar_true; zeros(3,1); zeros(3,1); zeros(3,1)];

%% ======================== FULL-STATE INDICES ===========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar_full = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;
idx_bmag = 19:21;

%% ======================== REDUCED-STATE INDICES ========================
idx_pt = 1:3;
idx_vt = 4:6;
idx_pbar = 7:8;

idx_dpt = 1:3;
idx_dvt = 4:6;
idx_dpbar = 7:8;

%% ======================== INITIALIZE REDUCED ESKF ======================
init_errors = struct();
init_errors.position = [3; -2; 1.5];
init_errors.velocity = [0.3; -0.2; 0.1];
init_errors.pbar = [0.02; -0.02];

x_init = ErrorStateKalmanFilter_reduced.createInitialState(p_tgt, v_tgt, pbar_true, init_errors);

init_sigma = struct();
init_sigma.position = 25.0;
init_sigma.velocity = 1.0;
init_sigma.pbar = 0.2;

P_init = ErrorStateKalmanFilter_reduced.createInitialCovariance(init_sigma);

eskf_params = struct();
eskf_params.dt_imu = dt_imu;
eskf_params.dt_eskf = dt_eskf;
eskf_params.R_b2c = R_b2c;
eskf_params.Qc = Qc_reduced;
eskf_params.Qd = Qd_reduced;
eskf_params.R_img = R_img;
eskf_params.R_radar = R_radar_init;
eskf_params.x_init = x_init;
eskf_params.P_init = P_init;
eskf_params.delay_steps = D;

eskf_red = ErrorStateKalmanFilter_reduced(eskf_params);

%% ======================== STORAGE FOR RESULTS ==========================
x_true_log = zeros(8, N_imu);
x_est_log = zeros(8, N_imu);
P_log = zeros(8, N_imu);
p_int_log = zeros(3, N_imu);

%% ======================== MAIN SIMULATION LOOP =========================
fprintf('Starting reduced ESKF simulation...\n');
fprintf('Total time: %.1f s, ESKF rate: %.0f Hz, Image rate: %.0f Hz, RADAR rate: %.1f Hz\n', ...
        t_total, 1/dt_eskf, 1/dt_image, 1/dt_radar);
fprintf('Image delay: %.0f ms (%d steps)\n', t_delay * 1000, D);
fprintf('Assumption: interceptor pose and velocity are known inputs.\n\n');

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

    %% -------------------- Truth propagation --------------------
    [x_true_full, p_int, v_int, p_tgt, v_tgt, omega_true, ~] = ...
        propagate_true_state(x_true_full, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar_full, idx_bgyr, idx_bacc, idx_bmag,p_tgt_center);

    q_true = x_true_full(idx_q);
    R_b2e_true = quat2rotm(q_true');
    x_true_reduced = [p_tgt; v_tgt; x_true_full(idx_pbar_full)];

    %% -------------------- Reduced ESKF prediction --------------------
    omega_accum = omega_accum + omega_true;
    a_accum = a_accum + zeros(3, 1);
    imu_count = imu_count + 1;

    eskf_update_counter = eskf_update_counter + 1;
    if eskf_update_counter >= eskf_sample_idx
        eskf_update_counter = 0;

        omega_avg = omega_accum / imu_count;
        a_avg = a_accum / imu_count;

        eskf_red.predict(omega_avg, a_avg, t, R_b2e_true, p_int, v_int);

        omega_accum = zeros(3, 1);
        a_accum = zeros(3, 1);
        imu_count = 0;
    end

    %% -------------------- Image correction --------------------
    image_update_counter = image_update_counter + 1;
    if image_update_counter >= image_sample_idx && k > D && useCam
        image_update_counter = 0;

        p_r_cam_true = R_b2c * R_b2e_true' * (-(p_int - p_tgt));
        if p_r_cam_true(3) > 2
            z_meas = [p_r_cam_true(1) / p_r_cam_true(3);
                      p_r_cam_true(2) / p_r_cam_true(3)] + sigma_img * randn(2, 1);
            eskf_red.correctImage(z_meas, D);
        end
    end

    %% -------------------- RADAR correction --------------------
    radar_update_counter = radar_update_counter + 1;
    if radar_update_counter >= radar_sample_idx && useRadar
        radar_update_counter = 0;

        [z_pos_ned0, z_vel_ned0, R_pos, R_vel] = ...
            emulate_radar_measurement_ned(p_tgt, v_tgt, radar_params);

        eskf_red.R_radar = blkdiag(R_pos, R_vel);
        z_radar = [z_pos_ned0; z_vel_ned0];
        eskf_red.correctRadar(z_radar);
    end

    %% -------------------- Store results --------------------
    x_true_log(:, k) = x_true_reduced;
    x_est_log(:, k) = eskf_red.x;
    P_log(:, k) = diag(eskf_red.P);
    p_int_log(:, k) = p_int;

    if mod(k, round(N_imu / 10)) == 0
        fprintf('Progress: %.0f%%\n', 100 * k / N_imu);
    end
end

fprintf('\nReduced simulation complete!\n');

%% ======================== PLOT RESULTS ================================
plot_eskf_results_reduced(t_imu, x_true_log, x_est_log, P_log, p_int_log, ...
                          idx_pt, idx_vt, idx_pbar, idx_dpt, idx_dvt, idx_dpbar);

%% ======================== STATISTICS ==================================
fprintf('\n=== Reduced ESKF Performance Statistics ===\n');

pos_error = x_true_log(idx_pt, :) - x_est_log(idx_pt, :);
pos_rmse = sqrt(mean(vecnorm(pos_error, 2, 1).^2));
fprintf('Target Position RMSE: %.3f m\n', pos_rmse);

vel_error = x_true_log(idx_vt, :) - x_est_log(idx_vt, :);
vel_rmse = sqrt(mean(vecnorm(vel_error, 2, 1).^2));
fprintf('Target Velocity RMSE: %.3f m/s\n', vel_rmse);

pbar_error = x_true_log(idx_pbar, :) - x_est_log(idx_pbar, :);
pbar_rmse = sqrt(mean(vecnorm(pbar_error, 2, 1).^2));
fprintf('Image Feature RMSE: %.4f\n', pbar_rmse);

fprintf('\n=== Final Error Statistics ===\n');
fprintf('Final Target Position Error: [%.3f, %.3f, %.3f] m\n', pos_error(:, end)');
fprintf('Final Target Velocity Error: [%.3f, %.3f, %.3f] m/s\n', vel_error(:, end)');
fprintf('Final Image Feature Error: [%.4f, %.4f]\n', pbar_error(:, end)');

function [pos_meas_ned, vel_meas_ned, R_pos, R_vel] = emulate_radar_measurement_ned(p_tgt_ned, v_tgt_ned, radar_params)
% EMULATE_RADAR_MEASUREMENT_NED Apply spherical radar noise directly in NED.
% This matches the reduced test setup where the radar is colocated with the
% simulation NED origin, so no geodetic conversion is required.

    p_tgt_ned = p_tgt_ned(:);
    v_tgt_ned = v_tgt_ned(:);

    x = p_tgt_ned(1);
    y = p_tgt_ned(2);
    z = p_tgt_ned(3);

    r_xy = hypot(x, y);
    r = norm(p_tgt_ned);

    theta = atan2(y, x);
    phi = atan2(-z, r_xy);

    r_m = max(r + radar_params.rmse_range * randn(), 0.01);
    theta_m = theta + radar_params.rmse_azimuth * randn();
    phi_m = phi + radar_params.rmse_elevation * randn();

    [R_pos, R_vel] = cartesian_covariance_from_spherical(r_m, theta_m, phi_m, radar_params);

    cos_phi_m = cos(phi_m);
    pos_meas_ned = [r_m * cos_phi_m * cos(theta_m);
                    r_m * cos_phi_m * sin(theta_m);
                   -r_m * sin(phi_m)];

    ur = p_tgt_ned / max(r, 1e-6);
    rdot = dot(v_tgt_ned, ur);

    if r_xy < 1e-6
        thetadot = 0;
        phidot = 0;
    else
        thetadot = (x * v_tgt_ned(2) - y * v_tgt_ned(1)) / (r_xy^2);
        rdot_xy = (x * v_tgt_ned(1) + y * v_tgt_ned(2)) / r_xy;
        phidot = (r_xy * (-v_tgt_ned(3)) - (-z) * rdot_xy) / (r_xy * max(r, 1e-6));
    end

    rdot_m = rdot + radar_params.rmse_doppler * randn();
    thetadot_m = thetadot + (radar_params.rmse_azimuth / radar_params.dt) * randn();
    phidot_m = phidot + (radar_params.rmse_elevation / radar_params.dt) * randn();

    cp = cos(phi_m);
    sp = sin(phi_m);
    ct = cos(theta_m);
    st = sin(theta_m);

    vel_meas_ned = [rdot_m * cp * ct - r_m * phidot_m * sp * ct - r_m * thetadot_m * cp * st;
                    rdot_m * cp * st - r_m * phidot_m * sp * st + r_m * thetadot_m * cp * ct;
                   -rdot_m * sp - r_m * phidot_m * cp];
end

function [R_pos, R_vel] = cartesian_covariance_from_spherical(r, theta, phi, radar_params)
    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    st = sin(theta);

    J = [cp * ct, -r * cp * st, -r * sp * ct;
         cp * st,  r * cp * ct, -r * sp * st;
         -sp,      0,           -r * cp];

    S_pos = diag([radar_params.rmse_range^2, ...
                  radar_params.rmse_azimuth^2, ...
                  radar_params.rmse_elevation^2]);

    S_vel = diag([radar_params.rmse_doppler^2, ...
                  (radar_params.rmse_azimuth / radar_params.dt)^2, ...
                  (radar_params.rmse_elevation / radar_params.dt)^2]);

    R_pos = 0.5 * (J * S_pos * J' + (J * S_pos * J')');
    R_vel = 0.5 * (J * S_vel * J' + (J * S_vel * J')');
end
