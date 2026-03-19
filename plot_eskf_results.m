function plot_eskf_results(t, x_true, x_est, P_diag, ...
                           idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc, idx_bmag, ...
                           idx_dtheta, idx_dpr, idx_dvr, idx_dpbar, idx_dbgyr, idx_dbacc, idx_dbmag)
% PLOT_ESKF_RESULTS Plot Error-State Kalman Filter estimation results
%
% Inputs:
%   t         - Time vector
%   x_true    - True nominal state history (21 x N)
%   x_est     - Estimated nominal state history (21 x N)
%   P_diag    - Diagonal of error covariance matrix history (20 x N)
%   idx_*     - Nominal state indices (21-state)
%   idx_d*    - Error state indices (20-state)

    N = length(t);
    
    %% ==================== MAIN STATE ESTIMATION FIGURE ====================
    figure('Name', 'ESKF State Estimation', 'Position', [100, 100, 1400, 900]);
    
    %% Relative Position - All 3 Components
    subplot(3, 2, 1);
    colors_true = {'b-', 'g-', 'm-'};
    colors_est = {'b--', 'g--', 'm--'};
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
        error_i = x_true(idx_pr(i), :) - x_est(idx_pr(i), :);
        sigma_i = sqrt(P_diag(idx_dpr(i), :));
        plot(t, error_i, 'LineWidth', 1.5);
    end
    % Add 3-sigma bounds for last component
    sigma_z = sqrt(P_diag(idx_dpr(3), :));
    plot(t, 3*sigma_z, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_z, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Position Estimation Error with 3σ bounds');
    legend('Error X', 'Error Y', 'Error Z', '±3σ', 'Location', 'best');
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
        error_i = x_true(idx_vr(i), :) - x_est(idx_vr(i), :);
        plot(t, error_i, 'LineWidth', 1.5);
    end
    % Add 3-sigma bounds
    sigma_vz = sqrt(P_diag(idx_dvr(3), :));
    plot(t, 3*sigma_vz, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_vz, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (m/s)');
    title('Velocity Estimation Error with 3σ bounds');
    legend('Error X', 'Error Y', 'Error Z', '±3σ', 'Location', 'best');
    grid on;
    
    %% Image Feature - Both Components
    subplot(3, 2, 5);
    hold on;
    plot(t, x_true(idx_pbar(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(1), :), 'b--', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(2), :), 'g--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Normalized Coordinate');
    title('Normalized Image Features');
    legend('True p_x', 'True p_y', 'Est p_x', 'Est p_y', 'Location', 'best');
    grid on;
    
    subplot(3, 2, 6);
    hold on;
    error_px = x_true(idx_pbar(1), :) - x_est(idx_pbar(1), :);
    error_py = x_true(idx_pbar(2), :) - x_est(idx_pbar(2), :);
    plot(t, error_px, 'b-', 'LineWidth', 1.5);
    plot(t, error_py, 'g-', 'LineWidth', 1.5);
    % Add 3-sigma bounds
    sigma_py = sqrt(P_diag(idx_dpbar(2), :));
    plot(t, 3*sigma_py, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_py, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error');
    title('Image Feature Estimation Error with 3σ bounds');
    legend('Error p_x', 'Error p_y', '±3σ', 'Location', 'best');
    grid on;
    
    sgtitle('Error-State Kalman Filter - State Estimation Results');
    
    %% ==================== BIAS ESTIMATION FIGURE ====================
    figure('Name', 'ESKF Bias Estimation', 'Position', [150, 150, 1200, 600]);
    
    subplot(2, 2, 1);
    hold on;
    plot(t, x_true(idx_bgyr(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bgyr(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bgyr(3), :), 'm-', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(1), :), 'b--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(2), :), 'g--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bgyr(3), :), 'm--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Gyro Bias (rad/s)');
    title('Gyroscope Bias Estimation');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 2);
    hold on;
    for i = 1:3
        error_i = x_true(idx_bgyr(i), :) - x_est(idx_bgyr(i), :);
        plot(t, error_i, 'LineWidth', 1.5);
    end
    % Add 3-sigma bounds
    sigma_bgz = sqrt(P_diag(idx_dbgyr(3), :));
    plot(t, 3*sigma_bgz, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_bgz, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (rad/s)');
    title('Gyro Bias Estimation Error with 3σ bounds');
    legend('Error X', 'Error Y', 'Error Z', '±3σ', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 3);
    hold on;
    plot(t, x_true(idx_bacc(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bacc(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bacc(3), :), 'm-', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(1), :), 'b--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(2), :), 'g--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bacc(3), :), 'm--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Accel Bias (m/s^2)');
    title('Accelerometer Bias Estimation');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(2, 2, 4);
    hold on;
    for i = 1:3
        error_i = x_true(idx_bacc(i), :) - x_est(idx_bacc(i), :);
        plot(t, error_i, 'LineWidth', 1.5);
    end
    % Add 3-sigma bounds
    sigma_baz = sqrt(P_diag(idx_dbacc(3), :));
    plot(t, 3*sigma_baz, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_baz, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (m/s^2)');
    title('Accel Bias Estimation Error with 3σ bounds');
    legend('Error X', 'Error Y', 'Error Z', '±3σ', 'Location', 'best');
    grid on;
    
    sgtitle('ESKF - IMU Bias Estimation Results');
    
    %% ==================== MAGNETOMETER BIAS ESTIMATION FIGURE ====================
    figure('Name', 'ESKF Magnetometer Bias Estimation', 'Position', [175, 175, 1200, 400]);
    
    subplot(1, 2, 1);
    hold on;
    plot(t, x_true(idx_bmag(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bmag(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_true(idx_bmag(3), :), 'm-', 'LineWidth', 1.5);
    plot(t, x_est(idx_bmag(1), :), 'b--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bmag(2), :), 'g--', 'LineWidth', 1.5);
    plot(t, x_est(idx_bmag(3), :), 'm--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Mag Bias');
    title('Magnetometer Bias Estimation');
    legend('True X', 'True Y', 'True Z', 'Est X', 'Est Y', 'Est Z', 'Location', 'best');
    grid on;
    
    subplot(1, 2, 2);
    hold on;
    for i = 1:3
        error_i = x_true(idx_bmag(i), :) - x_est(idx_bmag(i), :);
        plot(t, error_i, 'LineWidth', 1.5);
    end
    % Add 3-sigma bounds
    sigma_bmz = sqrt(P_diag(idx_dbmag(3), :));
    plot(t, 3*sigma_bmz, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_bmz, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error');
    title('Mag Bias Estimation Error with 3\sigma bounds');
    legend('Error X', 'Error Y', 'Error Z', '\pm3\sigma', 'Location', 'best');
    grid on;
    
    sgtitle('ESKF - Magnetometer Bias Estimation Results');
    
    %% ==================== ATTITUDE ESTIMATION FIGURE ====================
    figure('Name', 'ESKF Attitude Estimation', 'Position', [200, 200, 1200, 500]);
    
    % Convert quaternions to Euler angles
    eul_true = zeros(3, N);
    eul_est = zeros(3, N);
    att_error = zeros(1, N);
    
    for i = 1:N
        eul_true(:, i) = quat2eul(x_true(idx_q, i)', 'ZYX')' * 180/pi;
        eul_est(:, i) = quat2eul(x_est(idx_q, i)', 'ZYX')' * 180/pi;
        
        % Compute attitude error angle
        q_true_i = x_true(idx_q, i);
        q_est_i = x_est(idx_q, i);
        q_err = quatmultiply(q_true_i', quatconj(q_est_i'))';
        att_error(i) = 2 * acosd(min(abs(q_err(1)), 1));
    end
    
    subplot(2, 3, 1);
    plot(t, eul_true(1, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(1, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Yaw (deg)');
    title('Yaw Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    plot(t, eul_true(2, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(2, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Pitch (deg)');
    title('Pitch Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    plot(t, eul_true(3, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(3, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Roll (deg)');
    title('Roll Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    % Attitude errors with covariance bounds
    subplot(2, 3, 4);
    eul_error_yaw = wrapTo180(eul_true(1, :) - eul_est(1, :));
    sigma_theta_z = sqrt(P_diag(idx_dtheta(3), :)) * 180/pi;  % Convert to deg
    plot(t, eul_error_yaw, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, 3*sigma_theta_z, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_theta_z, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (deg)');
    title('Yaw Error with 3σ bounds'); grid on;
    
    subplot(2, 3, 5);
    eul_error_pitch = wrapTo180(eul_true(2, :) - eul_est(2, :));
    sigma_theta_y = sqrt(P_diag(idx_dtheta(2), :)) * 180/pi;
    plot(t, eul_error_pitch, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, 3*sigma_theta_y, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_theta_y, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (deg)');
    title('Pitch Error with 3σ bounds'); grid on;
    
    subplot(2, 3, 6);
    eul_error_roll = wrapTo180(eul_true(3, :) - eul_est(3, :));
    sigma_theta_x = sqrt(P_diag(idx_dtheta(1), :)) * 180/pi;
    plot(t, eul_error_roll, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, 3*sigma_theta_x, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_theta_x, 'k--', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (deg)');
    title('Roll Error with 3σ bounds'); grid on;
    
    sgtitle('ESKF - Attitude Estimation Results');
    
    %% ==================== ERROR COVARIANCE EVOLUTION FIGURE ====================
    figure('Name', 'ESKF Covariance Evolution', 'Position', [250, 250, 1000, 600]);
    
    subplot(2, 3, 1);
    semilogy(t, sqrt(P_diag(idx_dtheta(1), :))*180/pi, 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dtheta(2), :))*180/pi, 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dtheta(3), :))*180/pi, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma (deg)');
    title('Attitude σ'); legend('X', 'Y', 'Z'); grid on;
    
    subplot(2, 3, 2);
    semilogy(t, sqrt(P_diag(idx_dpr(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dpr(2), :)), 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dpr(3), :)), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma (m)');
    title('Position σ'); legend('X', 'Y', 'Z'); grid on;
    
    subplot(2, 3, 3);
    semilogy(t, sqrt(P_diag(idx_dvr(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dvr(2), :)), 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dvr(3), :)), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma (m/s)');
    title('Velocity σ'); legend('X', 'Y', 'Z'); grid on;
    
    subplot(2, 3, 4);
    semilogy(t, sqrt(P_diag(idx_dpbar(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dpbar(2), :)), 'g', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma');
    title('Image Feature σ'); legend('p_x', 'p_y'); grid on;
    
    subplot(2, 3, 5);
    semilogy(t, sqrt(P_diag(idx_dbgyr(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dbgyr(2), :)), 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dbgyr(3), :)), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma (rad/s)');
    title('Gyro Bias σ'); legend('X', 'Y', 'Z'); grid on;
    
    subplot(2, 3, 6);
    semilogy(t, sqrt(P_diag(idx_dbacc(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dbacc(2), :)), 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dbacc(3), :)), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma (m/s^2)');
    title('Accel Bias \sigma'); legend('X', 'Y', 'Z'); grid on;
    
    sgtitle('ESKF - Error Covariance Evolution (\sigma values)');
    
    %% ==================== MAG BIAS COVARIANCE FIGURE ====================
    figure('Name', 'ESKF Mag Bias Covariance', 'Position', [300, 300, 500, 400]);
    semilogy(t, sqrt(P_diag(idx_dbmag(1), :)), 'r', 'LineWidth', 1.5); hold on;
    semilogy(t, sqrt(P_diag(idx_dbmag(2), :)), 'g', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dbmag(3), :)), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\sigma');
    title('Magnetometer Bias \sigma Evolution'); 
    legend('X', 'Y', 'Z'); grid on;
end
