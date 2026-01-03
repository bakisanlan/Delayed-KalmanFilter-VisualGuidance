function verify_eskf_jacobians()
% VERIFY_ESKF_JACOBIANS Numerically verify ESKF Jacobians
%
% Compares analytical Fc/Fd with numerical derivatives

clear; clc;
fprintf('=== ESKF Jacobian Verification ===\n\n');

%% Setup
dt = 0.005;
epsilon = 1e-7;

% Camera rotation
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

%% Define test nominal state (18 states)
q = [0.9; 0.2; -0.3; 0.1];
q = q / norm(q);
p_r = [-30; 5; 2];
v_r = [2; -1; 0.5];
pbar = [0.1; -0.05];
b_gyr = [0.005; -0.003; 0.002];
b_acc = [0.02; -0.01; 0.015];

x_nominal = [q; p_r; v_r; pbar; b_gyr; b_acc];

% IMU measurements
omega_m = [0.1; 0.05; -0.02];
a_m = [-0.5; 0.3; 9.7];

fprintf('Test state:\n');
fprintf('  q = [%.4f, %.4f, %.4f, %.4f]\n', q(1), q(2), q(3), q(4));
fprintf('  p_r = [%.2f, %.2f, %.2f] m\n', p_r(1), p_r(2), p_r(3));
fprintf('  v_r = [%.2f, %.2f, %.2f] m/s\n', v_r(1), v_r(2), v_r(3));
fprintf('\n');

%% Compute analytical Jacobians
[Fc, Gc, Fd, Gd] = compute_eskf_jacobians(x_nominal, omega_m, a_m, dt, R_b2c);

fprintf('Analytical Fc size: %dx%d\n', size(Fc, 1), size(Fc, 2));
fprintf('Analytical Fd size: %dx%d\n\n', size(Fd, 1), size(Fd, 2));

%% Compute numerical Fd by perturbing error state
% The discrete transition is: δx(k+1) = Fd * δx(k)
% We compute this numerically by:
%   1. Apply small perturbation δx to nominal state
%   2. Propagate perturbed nominal state
%   3. Compute resulting error = perturbed_new - nominal_new
%   4. Fd(:,i) = error / perturbation

Fd_numerical = zeros(17, 17);

% Propagate nominal state (unperturbed)
x_nominal_new = propagateNominalState(x_nominal, omega_m, a_m, dt, R_b2c);

for i = 1:17
    % Create error state perturbation
    delta_x = zeros(17, 1);
    delta_x(i) = epsilon;
    
    % Apply perturbation to nominal state
    x_perturbed = injectError(x_nominal, delta_x);
    
    % Propagate perturbed state
    x_perturbed_new = propagateNominalState(x_perturbed, omega_m, a_m, dt, R_b2c);
    
    % Extract resulting error
    delta_x_new = extractError(x_nominal_new, x_perturbed_new);
    
    % Compute column of Fd
    Fd_numerical(:, i) = delta_x_new / epsilon;
end

%% Compare block by block
fprintf('=== Block-wise Comparison ===\n\n');

blocks = {
    'Fd_dtheta_dtheta', 1:3,   1:3;
    'Fd_dtheta_dbgyr',  1:3,   12:14;
    'Fd_dpr_dvr',       4:6,   7:9;
    'Fd_dvr_dtheta',    7:9,   1:3;
    'Fd_dvr_dbacc',     7:9,   15:17;
    'Fd_dpbar_dtheta',  10:11, 1:3;
    'Fd_dpbar_dpr',     10:11, 4:6;
    'Fd_dpbar_dvr',     10:11, 7:9;
    'Fd_dpbar_dpbar',   10:11, 10:11;
    'Fd_dpbar_dbgyr',   10:11, 12:14;
};

all_pass = true;
for b = 1:size(blocks, 1)
    name = blocks{b, 1};
    row_idx = blocks{b, 2};
    col_idx = blocks{b, 3};
    
    Fd_ana = Fd(row_idx, col_idx);
    Fd_num = Fd_numerical(row_idx, col_idx);
    
    error = max(abs(Fd_ana(:) - Fd_num(:)));
    
    if error > 1e-4
        status = '❌ MISMATCH';
        all_pass = false;
    elseif error > 1e-6
        status = '⚠️  SMALL DIFF';
    else
        status = '✅ OK';
    end
    
    fprintf('%20s: max_error = %.2e  %s\n', name, error, status);
    
    if error > 1e-4
        fprintf('   Analytical:\n');
        disp(Fd_ana);
        fprintf('   Numerical:\n');
        disp(Fd_num);
    end
end

fprintf('\n');
if all_pass
    fprintf('✅ All ESKF Jacobian blocks verified!\n');
else
    fprintf('❌ Some blocks have mismatches - check above.\n');
end

end

%% ==================== Helper Functions ====================

function x_new = propagateNominalState(x, omega_m, a_m, dt, R_b2c)
    % Propagate nominal state
    idx_q = 1:4; idx_pr = 5:7; idx_vr = 8:10;
    idx_pbar = 11:12; idx_bgyr = 13:15; idx_bacc = 16:18;
    
    q = x(idx_q);
    p_r = x(idx_pr);
    v_r = x(idx_vr);
    pbar = x(idx_pbar);
    b_gyr = x(idx_bgyr);
    b_acc = x(idx_bacc);
    
    g = 9.81;
    e3 = [0; 0; 1];
    
    % Corrected IMU
    omega = omega_m - b_gyr;
    a_body = a_m - b_acc;
    
    R_b2e = quat2rotm(q');
    
    % Quaternion update via exponential map
    omega_dt = omega * dt;
    dq = expQuat(omega_dt);
    q_new = quatmultiply(q', dq')';
    q_new = q_new / norm(q_new);
    
    % Velocity
    a_world = R_b2e * a_body + g * e3;
    v_r_new = v_r + a_world * dt;
    
    % Position
    p_r_new = p_r + 0.5 * (v_r + v_r_new) * dt;
    
    % Image features
    R_e2b = R_b2e';
    p_c = R_b2c * R_e2b * (-p_r);
    p_c_z = max(p_c(3), 0.1);
    v_c = R_b2c * R_e2b * v_r;
    omega_c = R_b2c * omega;
    
    pbar_x = pbar(1); pbar_y = pbar(2);
    Lv = [-1/p_c_z, 0, pbar_x/p_c_z; 0, -1/p_c_z, pbar_y/p_c_z];
    Lw = [pbar_x*pbar_y, -(1+pbar_x^2), pbar_y; (1+pbar_y^2), -pbar_x*pbar_y, -pbar_x];
    
    pbar_dot = Lv * v_c + Lw * omega_c;
    pbar_new = pbar + pbar_dot * dt;
    
    % Biases unchanged
    x_new = [q_new; p_r_new; v_r_new; pbar_new; b_gyr; b_acc];
end

function x_perturbed = injectError(x_nominal, delta_x)
    % Inject error state into nominal state
    idx_q = 1:4; idx_pr = 5:7; idx_vr = 8:10;
    idx_pbar = 11:12; idx_bgyr = 13:15; idx_bacc = 16:18;
    
    idx_dtheta = 1:3; idx_dpr = 4:6; idx_dvr = 7:9;
    idx_dpbar = 10:11; idx_dbgyr = 12:14; idx_dbacc = 15:17;
    
    x_perturbed = x_nominal;
    
    % Attitude: q_pert = q_nom ⊗ δq(δθ)
    delta_theta = delta_x(idx_dtheta);
    dq = expQuat(delta_theta);
    q_nom = x_nominal(idx_q);
    q_pert = quatmultiply(q_nom', dq')';
    q_pert = q_pert / norm(q_pert);
    x_perturbed(idx_q) = q_pert;
    
    % Other states: additive
    x_perturbed(idx_pr) = x_nominal(idx_pr) + delta_x(idx_dpr);
    x_perturbed(idx_vr) = x_nominal(idx_vr) + delta_x(idx_dvr);
    x_perturbed(idx_pbar) = x_nominal(idx_pbar) + delta_x(idx_dpbar);
    x_perturbed(idx_bgyr) = x_nominal(idx_bgyr) + delta_x(idx_dbgyr);
    x_perturbed(idx_bacc) = x_nominal(idx_bacc) + delta_x(idx_dbacc);
end

function delta_x = extractError(x_nominal, x_perturbed)
    % Extract error state: δx = x_perturbed ⊖ x_nominal
    idx_q = 1:4; idx_pr = 5:7; idx_vr = 8:10;
    idx_pbar = 11:12; idx_bgyr = 13:15; idx_bacc = 16:18;
    
    delta_x = zeros(17, 1);
    
    % Attitude error: δq = q_nom^{-1} ⊗ q_pert, then δθ = log(δq)
    q_nom = x_nominal(idx_q);
    q_pert = x_perturbed(idx_q);
    
    q_nom_inv = [q_nom(1); -q_nom(2:4)];  % Quaternion inverse (conjugate for unit quat)
    dq = quatmultiply(q_nom_inv', q_pert')';
    
    % Log map: δθ = 2 * atan2(||qv||, qw) * qv/||qv||
    qw = dq(1);
    qv = dq(2:4);
    qv_norm = norm(qv);
    
    if qv_norm < 1e-10
        delta_theta = [0; 0; 0];
    else
        theta = 2 * atan2(qv_norm, qw);
        delta_theta = theta * qv / qv_norm;
    end
    
    delta_x(1:3) = delta_theta;
    
    % Other states: simple subtraction
    delta_x(4:6) = x_perturbed(idx_pr) - x_nominal(idx_pr);
    delta_x(7:9) = x_perturbed(idx_vr) - x_nominal(idx_vr);
    delta_x(10:11) = x_perturbed(idx_pbar) - x_nominal(idx_pbar);
    delta_x(12:14) = x_perturbed(idx_bgyr) - x_nominal(idx_bgyr);
    delta_x(15:17) = x_perturbed(idx_bacc) - x_nominal(idx_bacc);
end

function dq = expQuat(delta_theta)
    % Exponential map: rotation vector to quaternion
    theta = norm(delta_theta);
    
    if theta < 1e-10
        dq = [1; 0; 0; 0];
    else
        u = delta_theta / theta;
        dq = [cos(theta/2); sin(theta/2) * u];
    end
end
