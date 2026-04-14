function plot_fgo_results(t, x_true, x_est, P_diag, p_int_log, cost_history)
% PLOT_FGO_RESULTS  Visualise Factor Graph Optimisation results.
%
% Inputs:
%   t            - Time vector (1 x N)
%   x_true       - True state  [p_t(3); v_t(3); pbar(2)]  (8 x N)
%   x_est        - Estimated   [p_t(3); v_t(3); pbar(2)]  (8 x N)
%   P_diag       - Marginal covariance diagonals [p_t(3); v_t(3)]  (6 x N)
%   p_int_log    - Interceptor position history (3 x N)
%   cost_history - GN cost per iteration (vector)

    idx_pt   = 1:3;
    idx_vt   = 4:6;
    idx_pbar = 7:8;

    colors = lines(3);

    % =====================================================================
    %  Figure 1: State estimation
    % =====================================================================
    figure('Name', 'FGO State Estimation', 'Position', [60 60 1500 950]);

    % --- Position ---
    subplot(3,2,1); hold on;
    for i = 1:3
        plot(t, x_true(idx_pt(i),:), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
        plot(t, x_est(idx_pt(i),:),  '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Position (m)');
    title('Target Position');
    legend('True X','Est X','True Y','Est Y','True Z','Est Z', 'Location','best');
    grid on;

    subplot(3,2,2); hold on;
    for i = 1:3
        err_i   = x_true(idx_pt(i),:) - x_est(idx_pt(i),:);
        sig_i   = sqrt(P_diag(i,:));
        plot(t, err_i,      '-',  'Color', colors(i,:), 'LineWidth', 1.5);
        plot(t,  3*sig_i,   '--', 'Color', colors(i,:), 'LineWidth', 1.0);
        plot(t, -3*sig_i,   '--', 'Color', colors(i,:), 'LineWidth', 1.0);
    end
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Position Error with 3\sigma Bounds');
    grid on;

    % --- Velocity ---
    subplot(3,2,3); hold on;
    for i = 1:3
        plot(t, x_true(idx_vt(i),:), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
        plot(t, x_est(idx_vt(i),:),  '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    title('Target Velocity');
    legend('True X','Est X','True Y','Est Y','True Z','Est Z', 'Location','best');
    grid on;

    subplot(3,2,4); hold on;
    for i = 1:3
        err_i = x_true(idx_vt(i),:) - x_est(idx_vt(i),:);
        sig_i = sqrt(P_diag(3+i,:));
        plot(t, err_i,      '-',  'Color', colors(i,:), 'LineWidth', 1.5);
        plot(t,  3*sig_i,   '--', 'Color', colors(i,:), 'LineWidth', 1.0);
        plot(t, -3*sig_i,   '--', 'Color', colors(i,:), 'LineWidth', 1.0);
    end
    xlabel('Time (s)'); ylabel('Error (m/s)');
    title('Velocity Error with 3\sigma Bounds');
    grid on;

    % --- Image features ---
    subplot(3,2,5); hold on;
    plot(t, x_true(idx_pbar(1),:), 'b-',  'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(1),:),  'b--', 'LineWidth', 1.5);
    plot(t, x_true(idx_pbar(2),:), 'g-',  'LineWidth', 1.5);
    plot(t, x_est(idx_pbar(2),:),  'g--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Normalised Coordinate');
    title('Image Features (derived from optimised p_t)');
    legend('True \bar{p}_x','Est \bar{p}_x', ...
           'True \bar{p}_y','Est \bar{p}_y','Location','best');
    grid on;

    % --- Convergence ---
    subplot(3,2,6); hold on;
    semilogy(1:length(cost_history), cost_history, 'ko-', ...
             'MarkerFaceColor', [0.2 0.5 0.9], 'LineWidth', 1.5);
    xlabel('Gauss-Newton Iteration'); ylabel('Cost  \Sigma ||r||^2');
    title('Optimisation Convergence');
    grid on;

    sgtitle('Factor Graph Optimisation — State Estimation Results', ...
            'FontSize', 14, 'FontWeight', 'bold');

    % =====================================================================
    %  Figure 2: Trajectory + covariance
    % =====================================================================
    figure('Name', 'FGO Trajectory and Covariance', ...
           'Position', [120 120 1400 650]);

    subplot(2,2,[1 3]);
    plot3(p_int_log(2,:), p_int_log(1,:), -p_int_log(3,:), ...
          'k-.', 'LineWidth', 1.2);
    hold on;
    plot3(x_true(idx_pt(2),:), x_true(idx_pt(1),:), -x_true(idx_pt(3),:), ...
          'b-', 'LineWidth', 1.8);
    plot3(x_est(idx_pt(2),:), x_est(idx_pt(1),:),  -x_est(idx_pt(3),:), ...
          'r--', 'LineWidth', 1.5);
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title('3D Trajectories');
    legend('Interceptor', 'True Target', 'FGO Estimate', 'Location','best');
    grid on; axis equal;

    subplot(2,2,2); hold on;
    for i = 1:3
        semilogy(t, sqrt(P_diag(i,:)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('\sigma (m)');
    title('Target Position Marginal \sigma');
    legend('X','Y','Z','Location','best'); grid on;

    subplot(2,2,4); hold on;
    for i = 1:3
        semilogy(t, sqrt(P_diag(3+i,:)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('\sigma (m/s)');
    title('Target Velocity Marginal \sigma');
    legend('v_x','v_y','v_z','Location','best'); grid on;

    sgtitle('FGO — Geometry and Marginal Covariance', ...
            'FontSize', 14, 'FontWeight', 'bold');
end
