%% compare_delay_methods.m
% Runs the Reduced ESKF twice under the same truth trajectory using two
% delay compensation methods and compares estimated pbar against truth.
%
% Methods compared:
%   'none'        - Naive: delayed pixel applied directly (no compensation)
%   'extrapolate' - Method 2: measurement forward-predicted to current time
%
% The RNG seed and all truth parameters are identical across both runs.

clear; clc; close all;

%% ======================== SIMULATION PARAMETERS ========================
dt_imu  = 1/200;
dt_eskf = 1/200;
dt_image = 1/30;
dt_radar = 1/0.5;
t_delay  = 200/1000;          % 80 ms delay
t_total  = 80;
D = round(t_delay / dt_imu);

useRadar = true;
useCam   = true;

t_imu = 0:dt_imu:t_total;
N_imu = length(t_imu);

%% ======================== PHYSICAL CONSTANTS ===========================
g  = 9.81;
e3 = [0; 0; 1];

%% ======================== CAMERA PARAMETERS ============================
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

%% ======================== SENSOR PARAMETERS ============================
sigma_img = 0.05;
R_img     = sigma_img^2 * eye(2);

sigma_tgt_rw = 1.5;
Qc_reduced   = sigma_tgt_rw^2 * eye(3);
Qd_reduced   = Qc_reduced * dt_eskf;

radar_params.rmse_range     = 3.0;
radar_params.rmse_azimuth   = deg2rad(1.3);
radar_params.rmse_elevation = deg2rad(3.3);
radar_params.rmse_doppler   = 0.5;
radar_params.dt             = dt_radar;

R_radar_init = diag([radar_params.rmse_range^2 * ones(1,3), ...
                     radar_params.rmse_doppler^2 * ones(1,3)]);

%% ======================== INITIAL CONDITIONS ===========================
p_int = [0; 0; 0];
v_int = [0; 1; -1];
yaw_init = 0;
q_true_init = eul2quat([yaw_init, 0, 0], 'ZYX')';

p_tgt = [160; 200; -50];
radius = 70;
p_tgt_center = p_tgt - [radius; 0; 0];
v_tgt = [0; 0; 0];

p_r_true  = p_int - p_tgt;
v_r_true  = v_int - v_tgt;
pbar_true0 = compute_image_features(p_r_true, q_true_init, R_b2c);
x_true_full_init = [q_true_init; p_r_true; v_r_true; pbar_true0; ...
                    zeros(3,1); zeros(3,1); zeros(3,1)];

%% ======================== FULL-STATE INDICES ===========================
idx_q = 1:4; idx_pr = 5:7; idx_vr = 8:10; idx_pbar_full = 11:12;
idx_bgyr = 13:15; idx_bacc = 16:18; idx_bmag = 19:21;

%% ======================== REDUCED-STATE INDICES ========================
idx_pt = 1:3; idx_vt = 4:6; idx_pbar = 7:8;
idx_dpt = 1:3; idx_dvt = 4:6; idx_dpbar = 7:8;

%% ======================== INITIAL ESKF STATE & COV ====================
init_errors.position = [3; -2; 1.5];
init_errors.velocity = [0.3; -0.2; 0.1];
init_errors.pbar     = [0.02; -0.02];

x_init = ErrorStateKalmanFilter_reduced.createInitialState( ...
    p_tgt, v_tgt, pbar_true0, init_errors);

init_sigma.position = 25.0;
init_sigma.velocity = 1.0;
init_sigma.pbar     = 0.2;
P_init = ErrorStateKalmanFilter_reduced.createInitialCovariance(init_sigma);

%% ======================== COMMON ESKF PARAMS ==========================
base_params.dt_imu      = dt_imu;
base_params.dt_eskf     = dt_eskf;
base_params.R_b2c       = R_b2c;
base_params.Qc          = Qc_reduced;
base_params.Qd          = Qd_reduced;
base_params.R_img       = R_img;
base_params.R_radar     = R_radar_init;
base_params.x_init      = x_init;
base_params.P_init      = P_init;
base_params.delay_steps = D;

%% ======================== METHODS TO COMPARE ==========================
methods      = {'none', 'extrapolate'};
method_names = {'None (naive)', 'Extrapolate (Method 2)'};
n_methods    = numel(methods);

% Storage: pbar estimate for each method
pbar_est_all = zeros(2, N_imu, n_methods);
pbar_true_log = zeros(2, N_imu);   % same for all runs

%% ======================== RUN EACH METHOD =============================
for m = 1:n_methods
    fprintf('\n=== Running method: %s ===\n', methods{m});
    rng(32);   % identical seed every run

    % Reset truth
    x_true_full = x_true_full_init;
    p_int_r = [0; 0; 0];
    v_int_r = [0; 1; -1];
    p_tgt_r = [160; 200; -50];
    v_tgt_r = [0; 0; 0];

    % Build filter
    params = base_params;
    params.delay_method = methods{m};
    eskf = ErrorStateKalmanFilter_reduced(params);

    % Truth geometry ring buffer (D+1 cols, oldest = col 1)
    q0 = x_true_full(idx_q);
    truth_hist_p_int = repmat(p_int_r,  1, D+1);
    truth_hist_p_tgt = repmat(p_tgt_r, 1, D+1);
    truth_hist_R_b2e = repmat(quat2rotm(q0'), 1, 1, D+1);

    % Counters
    eskf_update_counter  = 0;
    eskf_sample_idx      = round(dt_eskf / dt_imu);
    image_update_counter = 0;
    image_sample_idx     = round(dt_image / dt_imu);
    radar_update_counter = 0;
    radar_sample_idx     = round(dt_radar / dt_imu);
    omega_accum = zeros(3,1);
    a_accum     = zeros(3,1);
    imu_count   = 0;

    for k = 1:N_imu
        t = t_imu(k);

        %% Truth propagation
        [x_true_full, p_int_r, v_int_r, p_tgt_r, v_tgt_r, omega_true, ~] = ...
            propagate_true_state(x_true_full, p_int_r, v_int_r, p_tgt_r, v_tgt_r, ...
                                 t, dt_imu, g, e3, R_b2c, ...
                                 idx_q, idx_pr, idx_vr, idx_pbar_full, ...
                                 idx_bgyr, idx_bacc, idx_bmag, p_tgt_center);

        q_now       = x_true_full(idx_q);
        R_b2e_true  = quat2rotm(q_now');

        % Log truth pbar (same across all methods — written every time but identical)
        pbar_true_log(:, k) = x_true_full(idx_pbar_full);

        %% Update truth ring buffer
        truth_hist_p_int = [truth_hist_p_int(:, 2:end),  p_int_r];
        truth_hist_p_tgt = [truth_hist_p_tgt(:, 2:end),  p_tgt_r];
        truth_hist_R_b2e = cat(3, truth_hist_R_b2e(:,:,2:end), R_b2e_true);

        %% Prediction
        omega_accum = omega_accum + omega_true;
        imu_count   = imu_count + 1;
        eskf_update_counter = eskf_update_counter + 1;
        if eskf_update_counter >= eskf_sample_idx
            eskf_update_counter = 0;
            omega_avg = omega_accum / imu_count;
            a_avg     = a_accum / imu_count;
            eskf.predict(omega_avg, a_avg, t, R_b2e_true, p_int_r, v_int_r);
            omega_accum = zeros(3,1);
            a_accum     = zeros(3,1);
            imu_count   = 0;
        end

        %% Image correction (delayed measurement)
        image_update_counter = image_update_counter + 1;
        if image_update_counter >= image_sample_idx && k > D && useCam
            image_update_counter = 0;

            % Measurement was captured at k-D  (ring buffer col 1)
            p_int_d = truth_hist_p_int(:, 1);
            p_tgt_d = truth_hist_p_tgt(:, 1);
            R_b2e_d = truth_hist_R_b2e(:,:, 1);

            p_r_cam_d = R_b2c * R_b2e_d' * (-(p_int_d - p_tgt_d));
            if p_r_cam_d(3) > 2
                z_meas = [p_r_cam_d(1)/p_r_cam_d(3); ...
                          p_r_cam_d(2)/p_r_cam_d(3)] + sigma_img * randn(2,1);
                eskf.correctImage(z_meas, D);
                eskf.patchLastHistory();
            end
        end

        %% RADAR correction
        radar_update_counter = radar_update_counter + 1;
        if radar_update_counter >= radar_sample_idx && useRadar
            radar_update_counter = 0;
            [z_pos, z_vel, R_pos, R_vel] = ...
                emulate_radar_ned(p_tgt_r, v_tgt_r, radar_params);
            eskf.R_radar = blkdiag(R_pos, R_vel);
            eskf.correctRadar([z_pos; z_vel]);
            eskf.patchLastHistory();
        end

        %% Store
        pbar_est_all(:, k, m) = eskf.x(idx_pbar);

        if mod(k, round(N_imu/5)) == 0
            fprintf('  %.0f%%\n', 100*k/N_imu);
        end
    end
end

%% ======================== PLOT COMPARISON =============================
colors_method = [
    0.85, 0.33, 0.10;   % none        - orange-red
    0.00, 0.45, 0.74;   % extrapolate - blue
];
lw = 1.5;

figure('Name', 'pbar Delay Compensation Comparison', ...
       'Position', [80, 80, 1100, 560]);

pbar_labels = {'\bar{p}_x', '\bar{p}_y'};
for ax = 1:2
    subplot(1, 2, ax);
    hold on; grid on; box on;

    % True pbar (gray, thick)
    plot(t_imu, pbar_true_log(ax, :), 'k-', 'LineWidth', 2.2, ...
         'DisplayName', 'True');

    % Each method
    for m = 1:n_methods
        plot(t_imu, pbar_est_all(ax, :, m), ...
             'Color', colors_method(m,:), 'LineWidth', lw, ...
             'DisplayName', method_names{m});
    end

    xlabel('Time (s)', 'FontSize', 11);
    ylabel(pbar_labels{ax}, 'FontSize', 13, 'Interpreter', 'tex');
    title(sprintf('Image Feature %s', pbar_labels{ax}), ...
          'FontSize', 12, 'Interpreter', 'tex');
    legend('Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 10);
end

sgtitle(sprintf('Delay Compensation Comparison  (D = %d steps = %.0f ms)', ...
                D, t_delay*1000), 'FontSize', 13, 'FontWeight', 'bold');

%% ======================== PLOT ERROR COMPARISON =======================
figure('Name', 'pbar Error Comparison', ...
       'Position', [80, 680, 1100, 480]);

for ax = 1:2
    subplot(1, 2, ax);
    hold on; grid on; box on;
    yline(0, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');

    for m = 1:n_methods
        err = pbar_true_log(ax, :) - pbar_est_all(ax, :, m);
        plot(t_imu, err, 'Color', colors_method(m,:), 'LineWidth', lw, ...
             'DisplayName', method_names{m});
    end

    xlabel('Time (s)', 'FontSize', 11);
    ylabel(sprintf('Error in %s', pbar_labels{ax}), ...
           'FontSize', 11, 'Interpreter', 'tex');
    title(sprintf('%s Estimation Error', pbar_labels{ax}), ...
          'FontSize', 12, 'Interpreter', 'tex');
    legend('Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 10);
end

sgtitle('pbar Estimation Error by Delay Compensation Method', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% ======================== RMSE SUMMARY ================================
fprintf('\n=== pbar RMSE Comparison ===\n');
fprintf('%-26s  px_rmse    py_rmse\n', 'Method');
fprintf('%s\n', repmat('-', 1, 50));
for m = 1:n_methods
    err_px = pbar_true_log(1,:) - pbar_est_all(1,:,m);
    err_py = pbar_true_log(2,:) - pbar_est_all(2,:,m);
    fprintf('%-26s  %7.5f    %7.5f\n', method_names{m}, ...
            rms(err_px), rms(err_py));
end

%% ======================== HELPER FUNCTIONS ============================
function [pos_meas, vel_meas, R_pos, R_vel] = emulate_radar_ned(p_tgt, v_tgt, rp)
    p_tgt = p_tgt(:); v_tgt = v_tgt(:);
    x = p_tgt(1); y = p_tgt(2); z = p_tgt(3);
    r_xy = hypot(x,y); r = norm(p_tgt);
    theta = atan2(y,x); phi = atan2(-z, r_xy);
    r_m     = max(r + rp.rmse_range*randn(), 0.01);
    theta_m = theta + rp.rmse_azimuth*randn();
    phi_m   = phi   + rp.rmse_elevation*randn();
    [R_pos, R_vel] = cart_cov_sph(r_m, theta_m, phi_m, rp);
    cp = cos(phi_m); sp = sin(phi_m);
    ct = cos(theta_m); st = sin(theta_m);
    pos_meas = [r_m*cp*ct; r_m*cp*st; -r_m*sp];
    ur   = p_tgt / max(r, 1e-6);
    rdot = dot(v_tgt, ur);
    if r_xy < 1e-6; thetadot=0; phidot=0;
    else
        thetadot = (x*v_tgt(2)-y*v_tgt(1))/(r_xy^2);
        rdotxy   = (x*v_tgt(1)+y*v_tgt(2))/r_xy;
        phidot   = (r_xy*(-v_tgt(3))-(-z)*rdotxy)/(r_xy*max(r,1e-6));
    end
    rdot_m     = rdot     + rp.rmse_doppler*randn();
    thetadot_m = thetadot + (rp.rmse_azimuth/rp.dt)*randn();
    phidot_m   = phidot   + (rp.rmse_elevation/rp.dt)*randn();
    vel_meas = [rdot_m*cp*ct - r_m*phidot_m*sp*ct - r_m*thetadot_m*cp*st;
                rdot_m*cp*st - r_m*phidot_m*sp*st + r_m*thetadot_m*cp*ct;
               -rdot_m*sp   - r_m*phidot_m*cp];
end

function [R_pos, R_vel] = cart_cov_sph(r, theta, phi, rp)
    cp=cos(phi); sp=sin(phi); ct=cos(theta); st=sin(theta);
    J = [cp*ct, -r*cp*st, -r*sp*ct;
         cp*st,  r*cp*ct, -r*sp*st;
         -sp,    0,       -r*cp];
    S_pos = diag([rp.rmse_range^2, rp.rmse_azimuth^2, rp.rmse_elevation^2]);
    S_vel = diag([rp.rmse_doppler^2, (rp.rmse_azimuth/rp.dt)^2, ...
                  (rp.rmse_elevation/rp.dt)^2]);
    R_pos = 0.5*(J*S_pos*J' + (J*S_pos*J')');
    R_vel = 0.5*(J*S_vel*J' + (J*S_vel*J')');
end
