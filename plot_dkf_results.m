function plot_dkf_results(t, x_true, x_est, P_diag, idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc)
% PLOT_DKF_RESULTS Plot Delayed Kalman Filter estimation results
%
% Inputs:
%   t       - Time vector
%   x_true  - True state history (18 x N)
%   x_est   - Estimated state history (18 x N)
%   P_diag  - Diagonal of covariance matrix history (18 x N)
%   idx_*   - State indices

    %% ==================== MAIN STATE ESTIMATION FIGURE ====================
    figure('Name', 'DKF State Estimation', 'Position', [100, 100, 1400, 900]);
    
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
        plot(t, x_true(idx_pr(i), :) - x_est(idx_pr(i), :), 'LineWidth', 1.5);
    end
    % plot(t, vecnorm(x_true(idx_pr, :) - x_est(idx_pr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Position Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Location', 'best');
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
    % plot(t, vecnorm(x_true(idx_vr, :) - x_est(idx_vr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m/s)');
    title('Velocity Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Location', 'best');
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
    plot(t, x_true(idx_pbar(1), :) - x_est(idx_pbar(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2), :) - x_est(idx_pbar(2), :), 'g-', 'LineWidth', 1.5);
    % plot(t, vecnorm(x_true(idx_pbar, :) - x_est(idx_pbar, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error');
    title('Image Feature Estimation Error');
    legend('Error p_x', 'Error p_y', 'Location', 'best');
    grid on;
    
    sgtitle('Delayed Kalman Filter - State Estimation Results');
    
    %% ==================== BIAS ESTIMATION FIGURE ====================
    figure('Name', 'Bias Estimation', 'Position', [150, 150, 1200, 600]);
    
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
        plot(t, x_true(idx_bgyr(i), :) - x_est(idx_bgyr(i), :), 'LineWidth', 1.5);
    end
    % plot(t, vecnorm(x_true(idx_bgyr, :) - x_est(idx_bgyr, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (rad/s)');
    title('Gyro Bias Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Location', 'best');
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
        plot(t, x_true(idx_bacc(i), :) - x_est(idx_bacc(i), :), 'LineWidth', 1.5);
    end
    % plot(t, vecnorm(x_true(idx_bacc, :) - x_est(idx_bacc, :), 2, 1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (m/s^2)');
    title('Accel Bias Estimation Error');
    legend('Error X', 'Error Y', 'Error Z', 'Location', 'best');
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
    plot(t, eul_est(1, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Yaw (deg)');
    title('Yaw Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(1, 3, 2);
    plot(t, eul_true(2, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(2, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Pitch (deg)');
    title('Pitch Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    subplot(1, 3, 3);
    plot(t, eul_true(3, :), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, eul_est(3, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Roll (deg)');
    title('Roll Angle'); legend('True', 'Estimated', 'Location', 'best');
    grid on;
    
    sgtitle('Attitude Estimation Results');
end
