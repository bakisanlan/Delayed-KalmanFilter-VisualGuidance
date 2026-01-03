function verify_jacobians()
% VERIFY_JACOBIANS Compare analytical Jacobians with numerical (finite difference)
%
% This function computes both analytical and numerical Jacobians and compares
% them to identify any errors in the derivation.

clear; clc;

fprintf('=== Jacobian Verification ===\n\n');

%% Setup parameters
dt = 0.001;           % Time step
g = 9.81;
e3 = [0; 0; 1];

% Camera rotation
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

% State indices
idx_q = 1:4;
idx_pr = 5:7;
idx_vr = 8:10;
idx_pbar = 11:12;
idx_bgyr = 13:15;
idx_bacc = 16:18;

%% Define a test state
q_true = [1; 0.4; -0.2; 0.25];
q_true = q_true / norm(q_true);

p_r = [-30; 5; 2];          % Relative position
v_r = [2; -1; 0.5];         % Relative velocity
pbar = [0.1; -0.05];        % Image features
b_gyr = [0.005; -0.003; 0.002];
b_acc = [0.02; -0.01; 0.015];

% Assemble state
x = [q_true; p_r; v_r; pbar; b_gyr; b_acc];

% IMU measurements (before bias removal)
omega_meas = [0.1; 0.05; -0.02];
a_meas = [-0.5; 0.3; 9.7];

% Corrected values
% omega = omega_meas - b_gyr;
% a_body = a_meas - b_acc;

% Rotation matrix
R_b2e = quat2rotm(q_true');

% Depth in camera frame
p_r_cam = R_b2c * R_b2e' * (-p_r);
p_zc = max(p_r_cam(3), 0.1);

fprintf('Test conditions:\n');
fprintf('  p_zc (depth) = %.2f m\n', p_zc);
fprintf('  omega = [%.4f, %.4f, %.4f] rad/s\n', omega_meas(1), omega_meas(2), omega_meas(3));
fprintf('  a_body = [%.4f, %.4f, %.4f] m/s²\n\n', a_meas(1), a_meas(2), a_meas(3));

%% Compute analytical Jacobian
[F_analytical, G_analytical] = compute_jacobians(x, omega_meas, a_meas, dt, R_b2c, ...
                                                 idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);

%% Compute numerical Jacobian using finite differences
epsilon = 1e-7;
F_numerical = zeros(18, 18);

for j = 1:18
    % Perturb state
    x_plus = x;
    x_minus = x;
    x_plus(j) = x_plus(j) + epsilon;
    x_minus(j) = x_minus(j) - epsilon;
    
    % Normalize quaternion if perturbing quaternion states
    % if j <= 4
    %     x_plus(1:4) = x_plus(1:4) / norm(x_plus(1:4));
    %     x_minus(1:4) = x_minus(1:4) / norm(x_minus(1:4));
    % end
    
    % Propagate both states
    x_next_plus  = propagate_state_for_jacobian(x_plus,  omega_meas, a_meas, dt, R_b2c, g, e3);
    x_next_minus = propagate_state_for_jacobian(x_minus, omega_meas, a_meas, dt, R_b2c, g, e3);
    
    % Central difference
    F_numerical(:, j) = (x_next_plus - x_next_minus) / (2 * epsilon);
end

%% Compare Jacobians
fprintf('=== F Matrix Comparison ===\n\n');

% Define block names for better output
blocks = {
    'F_q_q',     idx_q,    idx_q;
    'F_q_bgyr',  idx_q,    idx_bgyr;
    'F_pr_pr',   idx_pr,   idx_pr;
    'F_pr_vr',   idx_pr,   idx_vr;
    'F_vr_q',    idx_vr,   idx_q;
    'F_vr_vr',   idx_vr,   idx_vr;
    'F_vr_bacc', idx_vr,   idx_bacc;
    'F_pbar_q',  idx_pbar, idx_q;
    'F_pbar_pr', idx_pbar, idx_pr;     % <-- Check if this should be non-zero!
    'F_pbar_vr', idx_pbar, idx_vr;
    'F_pbar_pbar', idx_pbar, idx_pbar;
    'F_pbar_bgyr', idx_pbar, idx_bgyr;
    'F_bgyr_bgyr', idx_bgyr, idx_bgyr;
    'F_bacc_bacc', idx_bacc, idx_bacc;
};

max_errors = zeros(size(blocks, 1), 1);

for b = 1:size(blocks, 1)
    name = blocks{b, 1};
    row_idx = blocks{b, 2};
    col_idx = blocks{b, 3};
    
    F_ana_block = F_analytical(row_idx, col_idx);
    F_num_block = F_numerical(row_idx, col_idx);
    
    error = abs(F_ana_block - F_num_block);
    max_error = max(error(:));
    max_errors(b) = max_error;
    
    if max_error > 1e-4
        status = '❌ MISMATCH';
    elseif max_error > 1e-6
        status = '⚠️  SMALL DIFF';
    else
        status = '✅ OK';
    end
    
    fprintf('%12s: max_error = %.2e  %s\n', name, max_error, status);
    
    % Print details for mismatches
    fprintf('    Analytical:\n');
    disp(F_ana_block);
    fprintf('    Numerical:\n');
    disp(F_num_block);
end

fprintf('\n');

%% Summary
if max(max_errors) > 1e-4
    fprintf('⚠️  Some Jacobian blocks have significant errors!\n');
    fprintf('   Check the blocks marked with MISMATCH above.\n');
else
    fprintf('✅ All Jacobian blocks match within tolerance.\n');
end

end

%% Helper function: State propagation (matches DKF predict)
function x_next = propagate_state_for_jacobian(x, omega_meas, a_body_meas, dt, R_b2c, g, e3)
    % Extract states
    q = x(1:4);
    p_r = x(5:7);
    v_r = x(8:10);
    pbar = x(11:12);
    % Biases don't change (random walk handled separately)
    b_gyr = x(13:15);
    b_acc = x(16:18);

    % Bias removal from IMU measurement
    omega  = omega_meas - b_gyr;
    a_body = a_body_meas - b_acc;
    
    % Rotation matrix
    R_b2e = quat2rotm(q');
    
    % Quaternion propagation
    wx = omega(1); wy = omega(2); wz = omega(3);
    M_dq = [1,        -wx*dt/2, -wy*dt/2, -wz*dt/2;
            wx*dt/2,   1,        wz*dt/2, -wy*dt/2;
            wy*dt/2,  -wz*dt/2,  1,        wx*dt/2;
            wz*dt/2,   wy*dt/2, -wx*dt/2,  1];
    q_next = M_dq * q;
    % q_next = q_next / norm(q_next);
    
    % Velocity propagation
    a_world = R_b2e * a_body + g * e3;
    v_r_next = v_r + a_world * dt;
    
    % Position propagation (trapezoidal)
    p_r_next = p_r + 0.5 * (v_r + v_r_next) * dt;
    
    % Image feature propagation
    p_r_cam = R_b2c * R_b2e' * (-p_r);
    p_zc = max(p_r_cam(3), 0.1);
    
    pbar_x = pbar(1); pbar_y = pbar(2);
    
    % Lv and Lw matrices
    Lv = [-1/p_zc,     0,    pbar_x/p_zc;
           0,     -1/p_zc,   pbar_y/p_zc];
    v_cam = R_b2c * R_b2e' * v_r;
    
    Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;
          (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
    omega_cam = R_b2c * omega;
    
    pbar_next = pbar + (Lv * v_cam + Lw * omega_cam) * dt;
    
    % Assemble next state
    x_next = [q_next; p_r_next; v_r_next; pbar_next; b_gyr; b_acc];
end
