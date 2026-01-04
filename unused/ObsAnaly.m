%% Observability Analysis of DKF using Gramian SVD
% Computes the observability Gramian numerically and analyzes 
% state observability using SVD decomposition.
%
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.

clear; clc; close all;

%% ======================== SIMULATION PARAMETERS ========================
dt_imu = 1/200;           % IMU update rate: 200 Hz
N_steps = 4000;            % Number of steps for Gramian computation
g = 9.81;                 % Gravity
e3 = [0; 0; 1];           % Unit vector z

%% ======================== CAMERA PARAMETERS ========================
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

%% ======================== STATE INDICES ========================
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;

%% ======================== INITIAL CONDITIONS ========================
% Interceptor initial state
p_int = [1000; 0; -40];
v_int = [0; 0; 0];
yaw_init = 0;
q_true = eul2quat([yaw_init, 0, 0], 'ZYX')';

% Target initial state
p_tgt = [3000; 0; -40];
v_tgt = [0; 0; 0];

% Relative state
p_r_true = p_int - p_tgt;
v_r_true = v_int - v_tgt;

% IMU biases
b_gyr_true = 0*[0.005; -0.003; 0.002];
b_acc_true = 0*[0.02; -0.01; 0.015];

% Compute initial image features
pbar_true = compute_image_features(p_r_true, q_true, R_b2c);

% Assemble initial true state
x_true = [q_true; p_r_true; v_r_true; pbar_true; b_gyr_true; b_acc_true];

%% ======================== MEASUREMENT MATRIX H ========================
H = zeros(5, 18);
H(1:2, idx_pbar) = eye(2);
H(3:5,idx_pr) = eye(3);

%% ======================== OBSERVABILITY GRAMIAN COMPUTATION ========================
% Initialize Gramian
L_k = zeros(18, 18);

% Initialize state transition product (Phi_k = F_{k-1} * F_{k-2} * ... * F_0)
Phi_k = eye(18);

% Store F matrices for analysis
F_history = zeros(18, 18, N_steps);

fprintf('Computing Observability Gramian over %d steps...\n', N_steps);

for k = 1:N_steps
    t = (k-1) * dt_imu;
    
    %% Propagate true state to get current trajectory point
    [x_true, p_int, v_int, p_tgt, v_tgt, omega_true, a_body_true] = ...
        propagate_true_state(x_true, p_int, v_int, p_tgt, v_tgt, ...
                             t, dt_imu, g, e3, R_b2c, ...
                             idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    %% Extract states for Jacobian computation
    q = x_true(idx_q);
    p_r = x_true(idx_pr);
    v_r = x_true(idx_vr);
    pbar = x_true(idx_pbar);
    
    % Compute rotation matrix
    R_b2e = quat2rotm(q');
    
    % Compute depth in camera frame
    p_r_cam = R_b2c * R_b2e' * (-p_r);
    p_zc = max(p_r_cam(3), 0.1);

    if p_zc < 2
        disp('stopped')
        return 
    end    
    
    %% Compute state transition Jacobian F_k
    [F_k, ~] = compute_jacobians(x_true, omega_true, a_body_true, dt_imu, R_b2c, ...
                                 idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
    
    % Store for analysis
    F_history(:,:,k) = F_k;
    
    %% Update state transition product: Phi_k = F_k * Phi_{k-1}
    Phi_k = F_k * Phi_k;
    
    %% Accumulate Observability Gramian: L = sum( Phi_k' * H' * H * Phi_k )
    L_k = L_k + Phi_k' * (H' * H) * Phi_k;
    
    %% Progress
    if mod(k, 50) == 0
        fprintf('  Step %d / %d\n', k, N_steps);
    end
end

fprintf('Gramian computation complete.\n\n');

%% ======================== SVD ANALYSIS ========================
[svd_U, svd_S, svd_V] = svd(L_k);

% Extract singular values
singular_values = diag(svd_S);
fprintf('Singular Values of Observability Gramian:\n');
for i = 1:18
    fprintf('  σ_%d = %.4e\n', i, singular_values(i));
end

% Condition number
cond_number = singular_values(1) / singular_values(end);
fprintf('\nCondition Number: %.4e\n', cond_number);

%% ======================== STATE OBSERVABILITY MEASURE ========================
% Finding each state's observabiltiy by examining basis vector contribution
obs_state = zeros(18,1);
for i = 1:18
    sum_sq = 0;
    for j = 1:18
        sum_sq = sum_sq + svd_S(j,j)^2 * svd_U(i,j)^2;
    end
    obs_state(i) = sqrt(sum_sq);
end

fprintf('\nState Observability Measures:\n');
state_names = {'q0', 'q1', 'q2', 'q3', 'p_rx', 'p_ry', 'p_rz', ...
               'v_rx', 'v_ry', 'v_rz', 'pbar_x', 'pbar_y', ...
               'b_gyr_x', 'b_gyr_y', 'b_gyr_z', 'b_acc_x', 'b_acc_y', 'b_acc_z'};
for i = 1:18
    fprintf('  %8s: %.4e\n', state_names{i}, obs_state(i));
end

%% ======================== VISUALIZATION ========================

% ---- 1. SVD U matrix heatmap ----
M = svd_U;
A = abs(M);

figure('Color','w', 'Position', [100, 100, 1000, 800]);
imagesc(A, [0 1]);
axis equal tight ij
colormap("sky");
colorbar('Ticks', 0:0.2:1);

% Add grid lines
hold on
[n, ~] = size(M);
for k = 0.5 : 3 : (n + 0.5)
    plot([0.5 (n + 0.5)], [k k], 'k', 'LineWidth', 2);
    plot([k k], [0.5 (n + 0.5)], 'k', 'LineWidth', 2);
end
for k = 0.5 : 1 : (n + 0.5)
    plot([0.5 (n + 0.5)], [k k], 'k', 'LineWidth', 0.5);
    plot([k k], [0.5 (n + 0.5)], 'k', 'LineWidth', 0.5);
end
hold off

% Row labels (states)
rowLabels = {'$q_0$', '$q_1$', '$q_2$', '$q_3$', ...
             '$p_{rx}$', '$p_{ry}$', '$p_{rz}$', ...
             '$v_{rx}$', '$v_{ry}$', '$v_{rz}$', ...
             '$\bar{p}_x$', '$\bar{p}_y$', ...
             '$b_{\omega x}$', '$b_{\omega y}$', '$b_{\omega z}$', ...
             '$b_{ax}$', '$b_{ay}$', '$b_{az}$'};

% Column labels (singular values)
colLabels = arrayfun(@(v) sprintf('$%.2E$', v), diag(svd_S)', 'UniformOutput', false);

% Flag cells with A(i,j) > 0.1
threshold = 0.1;
[row, col] = find(A > threshold);
hold on
plot(col, row, 'ko', 'MarkerSize', 6, 'LineWidth', 1.2);
hold off

set(gca, 'XTick', 1:n, 'XTickLabel', colLabels, ...
         'YTick', 1:n, 'YTickLabel', rowLabels, ...
         'TickLabelInterpreter', 'latex', 'FontSize', 12);

% Add observability measures on left
ax = gca;
xLeft = 0.5;
xShift = -3.5;

text(xLeft + xShift + 2, 0, 'Obs. Measure', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
     'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');

for r = 1:n
    text(xLeft + xShift, r, sprintf('%.2E', obs_state(r)), ...
         'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
         'FontSize', 11, 'Interpreter', 'latex');
end

% Annotation for minimum observability
pos = ax.Position;
xBox = pos(1) + pos(3) - 0.05;
yBox = pos(2) + pos(4)/2 - 0.03;

maxS = max(singular_values);
minOBS = 1.0E-16 * maxS;
exp10 = floor(log10(minOBS));
mantissa = minOBS / 10^exp10;
latexStr = sprintf('$\\mathrm{Min\\ obs.\\ measure} = %.2f\\times 10^{%d}$', mantissa, exp10);

annotation('textbox', [xBox yBox 0.15 0.06], 'String', latexStr, ...
           'Units', 'normalized', 'EdgeColor', 'none', 'FontSize', 12, ...
           'FontWeight', 'bold', 'Interpreter', 'latex', 'HorizontalAlignment', 'left');

title('DKF Observability Analysis - SVD of Gramian', 'FontSize', 14);

%% ---- 2. Singular Values Bar Plot ----
figure('Color', 'w', 'Position', [150, 150, 800, 400]);
bar(log10(singular_values));
xlabel('Singular Value Index', 'FontSize', 12);
ylabel('log_{10}(\sigma)', 'FontSize', 12);
title('Singular Values of Observability Gramian (log scale)', 'FontSize', 14);
grid on;
xticks(1:18);

%% ---- 3. State Observability Bar Plot ----
figure('Color', 'w', 'Position', [200, 200, 1000, 400]);
bar(log10(obs_state));
xlabel('State Index', 'FontSize', 12);
ylabel('log_{10}(Observability Measure)', 'FontSize', 12);
title('State-wise Observability Measure', 'FontSize', 14);
grid on;
xticks(1:18);
xticklabels(state_names);
xtickangle(45);

fprintf('\n=== Observability Analysis Complete ===\n');
