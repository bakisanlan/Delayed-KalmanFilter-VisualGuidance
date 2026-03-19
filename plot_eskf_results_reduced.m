function plot_eskf_results_reduced(t, x_true, x_est, P_diag, p_int_log, ...
                                   idx_pt, idx_vt, idx_pbar, idx_dpt, idx_dvt, idx_dpbar)
% PLOT_ESKF_RESULTS_REDUCED Plot reduced-state ESKF estimation results.

    colors = lines(3);

    figure('Name', 'Reduced ESKF State Estimation', 'Position', [100, 100, 1400, 900]);

    subplot(3, 2, 1);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_pt(i), :), '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, x_est(idx_pt(i), :), '--', 'Color', colors(i, :), 'LineWidth', 1.5);
    end
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Target Position');
    legend('True X', 'Est X', 'True Y', 'Est Y', 'True Z', 'Est Z', 'Location', 'best');
    grid on;

    subplot(3, 2, 2);
    hold on;
    for i = 1:3
        err_i = x_true(idx_pt(i), :) - x_est(idx_pt(i), :);
        sigma_i = sqrt(P_diag(idx_dpt(i), :));
        plot(t, err_i, '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, 3 * sigma_i, '--', 'Color', colors(i, :), 'LineWidth', 1.0);
        plot(t, -3 * sigma_i, '--', 'Color', colors(i, :), 'LineWidth', 1.0);
    end
    xlabel('Time (s)');
    ylabel('Error (m)');
    title('Position Error with 3\sigma Bounds');
    legend('Err X', '+3\sigma X', '-3\sigma X', ...
           'Err Y', '+3\sigma Y', '-3\sigma Y', ...
           'Err Z', '+3\sigma Z', '-3\sigma Z', 'Location', 'best');
    grid on;

    subplot(3, 2, 3);
    hold on;
    for i = 1:3
        plot(t, x_true(idx_vt(i), :), '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, x_est(idx_vt(i), :), '--', 'Color', colors(i, :), 'LineWidth', 1.5);
    end
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Target Velocity');
    legend('True X', 'Est X', 'True Y', 'Est Y', 'True Z', 'Est Z', 'Location', 'best');
    grid on;

    subplot(3, 2, 4);
    hold on;
    for i = 1:3
        err_i = x_true(idx_vt(i), :) - x_est(idx_vt(i), :);
        sigma_i = sqrt(P_diag(idx_dvt(i), :));
        plot(t, err_i, '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, 3 * sigma_i, '--', 'Color', colors(i, :), 'LineWidth', 1.0);
        plot(t, -3 * sigma_i, '--', 'Color', colors(i, :), 'LineWidth', 1.0);
    end
    xlabel('Time (s)');
    ylabel('Error (m/s)');
    title('Velocity Error with 3\sigma Bounds');
    legend('Err X', '+3\sigma X', '-3\sigma X', ...
           'Err Y', '+3\sigma Y', '-3\sigma Y', ...
           'Err Z', '+3\sigma Z', '-3\sigma Z', 'Location', 'best');
    grid on;

    subplot(3, 2, 5);
    hold on;
    plot(t, x_true(idx_pbar(1), :), 'b-', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(1), :), 'b--', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2), :), 'g-', 'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(2), :), 'g--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Normalized Coordinate');
    title('Image Features');
    legend('True p_x', 'Est p_x', 'True p_y', 'Est p_y', 'Location', 'best');
    grid on;

    subplot(3, 2, 6);
    hold on;
    err_px = x_true(idx_pbar(1), :) - x_est(idx_pbar(1), :);
    err_py = x_true(idx_pbar(2), :) - x_est(idx_pbar(2), :);
    sigma_px = sqrt(P_diag(idx_dpbar(1), :));
    sigma_py = sqrt(P_diag(idx_dpbar(2), :));
    plot(t, err_px, 'b-', 'LineWidth', 1.5);
    plot(t, 3 * sigma_px, 'b--', 'LineWidth', 1.0);
    plot(t, -3 * sigma_px, 'b--', 'LineWidth', 1.0);
    plot(t, err_py, 'g-', 'LineWidth', 1.5);
    plot(t, 3 * sigma_py, 'g--', 'LineWidth', 1.0);
    plot(t, -3 * sigma_py, 'g--', 'LineWidth', 1.0);
    xlabel('Time (s)');
    ylabel('Error');
    title('Image Feature Error with 3\sigma Bounds');
    legend('Err p_x', '+3\sigma p_x', '-3\sigma p_x', ...
           'Err p_y', '+3\sigma p_y', '-3\sigma p_y', 'Location', 'best');
    grid on;

    sgtitle('Reduced ESKF - State Estimation Results');

    figure('Name', 'Reduced ESKF Trajectory and Covariance', 'Position', [150, 150, 1400, 700]);

    subplot(2, 2, [1 3]);
    plot3(p_int_log(2, :), p_int_log(1, :), -p_int_log(3, :), 'k-.', 'LineWidth', 1.2);
    hold on;
    plot3(x_true(idx_pt(2), :), x_true(idx_pt(1), :), -x_true(idx_pt(3), :), 'b-', 'LineWidth', 1.8);
    plot3(x_est(idx_pt(2), :), x_est(idx_pt(1), :), -x_est(idx_pt(3), :), 'r--', 'LineWidth', 1.5);
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Up (m)');
    title('3D Trajectories');
    legend('Interceptor', 'True Target', 'Estimated Target', 'Location', 'best');
    grid on;
    axis equal;

    subplot(2, 2, 2);
    hold on;
    for i = 1:3
        semilogy(t, sqrt(P_diag(idx_dpt(i), :)), 'Color', colors(i, :), 'LineWidth', 1.5);
    end
    xlabel('Time (s)');
    ylabel('\sigma (m)');
    title('Target Position Covariance');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;

    subplot(2, 2, 4);
    semilogy(t, sqrt(P_diag(idx_dvt(1), :)), 'r-', 'LineWidth', 1.5);
    hold on;
    semilogy(t, sqrt(P_diag(idx_dvt(2), :)), 'g-', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dvt(3), :)), 'b-', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dpbar(1), :)), 'm--', 'LineWidth', 1.5);
    semilogy(t, sqrt(P_diag(idx_dpbar(2), :)), 'c--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\sigma');
    title('Velocity and Image-Feature Covariance');
    legend('v_x', 'v_y', 'v_z', 'p_x', 'p_y', 'Location', 'best');
    grid on;

    sgtitle('Reduced ESKF - Geometry and Covariance');
end
