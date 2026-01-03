function test_F_pbar_q()
% Test the F_pbar_q Jacobian specifically
% Compare analytical formula with numerical derivative

clear; clc;
fprintf('=== F_pbar_q Jacobian Verification ===\n\n');

%% Setup
dt = 0.001;
epsilon = 1e-8;

% Camera rotation
R_c2b = [0 0 1; 1 0 0; 0 1 0];
R_b2c = R_c2b';

%% Test state
q = [0.9; 0.2; -0.3; 0.1];
q = q / norm(q);

p_r = [-30; 5; 2];
v_r = [2; -1; 0.5];
pbar = [0.1; -0.05];
omega = [0.1; 0.05; -0.02];  % Angular velocity

q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
pbar_x = pbar(1); pbar_y = pbar(2);

R_b2e = quat2rotm(q');

% Compute p_zc
p_r_cam = R_b2c * R_b2e' * (-p_r);
p_zc = max(p_r_cam(3), 0.1);

fprintf('Quaternion: [%.4f, %.4f, %.4f, %.4f]\n', q(1), q(2), q(3), q(4));
fprintf('v_r: [%.4f, %.4f, %.4f]\n', v_r(1), v_r(2), v_r(3));
fprintf('pbar: [%.4f, %.4f]\n', pbar(1), pbar(2));
fprintf('p_zc: %.4f\n\n', p_zc);

%% Numerical derivative (ground truth)
F_numerical = zeros(2, 4);

for i = 1:4
    q_plus = q; q_plus(i) = q_plus(i) + epsilon;
    q_minus = q; q_minus(i) = q_minus(i) - epsilon;
    
    pbar_plus = propagate_pbar(q_plus, v_r, pbar, omega, R_b2c, dt);
    pbar_minus = propagate_pbar(q_minus, v_r, pbar, omega, R_b2c, dt);
    
    F_numerical(:, i) = (pbar_plus - pbar_minus) / (2 * epsilon);
end

fprintf('Numerical ∂pbar/∂q:\n');
disp(F_numerical);

%% Analytical formula (current implementation)
% dR/dqi matrices
dR_dq0 = 2 * [0, -q3, q2; q3, 0, -q1; -q2, q1, 0];
dR_dq1 = 2 * [0, q2, q3; q2, -2*q1, -q0; q3, q0, -2*q1];
dR_dq2 = 2 * [-2*q2, q1, q0; q1, 0, q3; -q0, q3, -2*q2];
dR_dq3 = 2 * [-2*q3, -q0, q1; q0, -2*q3, q2; q1, q2, 0];

% Lv matrix
Lv = [-1/p_zc, 0, pbar_x/p_zc;
       0, -1/p_zc, pbar_y/p_zc];

% Current analytical: ∂(R'*v_r)/∂qi = (∂R/∂qi)' * v_r
dRt_v_dq0 = dR_dq0' * v_r;
dRt_v_dq1 = dR_dq1' * v_r;
dRt_v_dq2 = dR_dq2' * v_r;
dRt_v_dq3 = dR_dq3' * v_r;

J_pbar_raw = Lv * R_b2c * [dRt_v_dq0, dRt_v_dq1, dRt_v_dq2, dRt_v_dq3];

% Normalization correction
normalization_correction = eye(4) - q * q';
F_analytical = J_pbar_raw * normalization_correction * dt;

fprintf('Analytical (current):\n');
disp(F_analytical);

%% Alternative: Direct numerical derivative of v_cam
fprintf('=== Debugging v_cam dependency ===\n');
v_cam = R_b2c * R_b2e' * v_r;
fprintf('v_cam: [%.4f, %.4f, %.4f]\n', v_cam(1), v_cam(2), v_cam(3));

% Numerical ∂v_cam/∂q
dv_cam_dq_numerical = zeros(3, 4);
for i = 1:4
    q_plus = q; q_plus(i) = q_plus(i) + epsilon;
    q_minus = q; q_minus(i) = q_minus(i) - epsilon;
    
    R_plus = quat2rotm(q_plus');
    R_minus = quat2rotm(q_minus');
    
    v_cam_plus = R_b2c * R_plus' * v_r;
    v_cam_minus = R_b2c * R_minus' * v_r;
    
    dv_cam_dq_numerical(:, i) = (v_cam_plus - v_cam_minus) / (2 * epsilon);
end

fprintf('Numerical ∂v_cam/∂q:\n');
disp(dv_cam_dq_numerical);

% Analytical ∂v_cam/∂q = R_b2c * ∂(R')/∂q * v_r
dv_cam_dq_analytical = R_b2c * [dRt_v_dq0, dRt_v_dq1, dRt_v_dq2, dRt_v_dq3];
dv_cam_dq_corrected = dv_cam_dq_analytical * normalization_correction;

fprintf('Analytical ∂v_cam/∂q (with normalization):\n');
disp(dv_cam_dq_corrected);

fprintf('v_cam Jacobian error: %.2e\n\n', max(abs(dv_cam_dq_corrected(:) - dv_cam_dq_numerical(:))));

%% Check if p_zc also changes with q
fprintf('=== Checking p_zc dependency on q ===\n');
p_zc_derivatives = zeros(1, 4);
for i = 1:4
    q_plus = q; q_plus(i) = q_plus(i) + epsilon;
    q_minus = q; q_minus(i) = q_minus(i) - epsilon;
    
    R_plus = quat2rotm(q_plus');
    R_minus = quat2rotm(q_minus');
    
    p_r_cam_plus = R_b2c * R_plus' * (-p_r);
    p_r_cam_minus = R_b2c * R_minus' * (-p_r);
    
    p_zc_plus = p_r_cam_plus(3);
    p_zc_minus = p_r_cam_minus(3);
    
    p_zc_derivatives(i) = (p_zc_plus - p_zc_minus) / (2 * epsilon);
end

fprintf('Numerical ∂p_zc/∂q: [%.6f, %.6f, %.6f, %.6f]\n', p_zc_derivatives);

if max(abs(p_zc_derivatives)) > 1e-6
    fprintf('⚠️ p_zc DOES depend on q! Must include ∂Lv/∂p_zc * ∂p_zc/∂q\n\n');
end

%% Correct formula including p_zc dependency
fprintf('=== Full Jacobian including p_zc dependency ===\n');

% pbar_dot = Lv * v_cam = [-1/p_zc, 0, pbar_x/p_zc; 0, -1/p_zc, pbar_y/p_zc] * v_cam
% ∂pbar_dot/∂q = ∂Lv/∂q * v_cam + Lv * ∂v_cam/∂q
%              = ∂Lv/∂p_zc * ∂p_zc/∂q * v_cam + Lv * ∂v_cam/∂q

% ∂Lv/∂p_zc
dLv_dpzc = [1/p_zc^2, 0, -pbar_x/p_zc^2;
            0, 1/p_zc^2, -pbar_y/p_zc^2];

% ∂p_zc/∂q (numerical for now)
dpzc_dq = p_zc_derivatives;

% Additional term from p_zc dependency
term1 = dLv_dpzc * v_cam * dpzc_dq;  % 2x1 * 1x4 = 2x4

% v_cam term
term2 = Lv * dv_cam_dq_numerical;  % 2x3 * 3x4 = 2x4

F_full = (term1 + term2) * dt;

fprintf('Full analytical (including p_zc):\n');
disp(F_full);

fprintf('=== Final Errors ===\n');
fprintf('Current formula error:  %.2e\n', max(abs(F_analytical(:) - F_numerical(:))));
fprintf('Full formula error:     %.2e\n', max(abs(F_full(:) - F_numerical(:))));

end

%% Helper function
function pbar_next = propagate_pbar(q, v_r, pbar, omega, R_b2c, dt)
    R_b2e = quat2rotm(q');
    
    % Compute p_zc (need p_r, but we don't have it - use a fixed reference)
    % Actually for this test we fix p_r, so p_zc changes with q
    p_r = [-30; 5; 2];  % Same as in main function
    p_r_cam = R_b2c * R_b2e' * (-p_r);
    p_zc = max(p_r_cam(3), 0.1);
    
    pbar_x = pbar(1); pbar_y = pbar(2);
    
    Lv = [-1/p_zc, 0, pbar_x/p_zc;
           0, -1/p_zc, pbar_y/p_zc];
    v_cam = R_b2c * R_b2e' * v_r;
    
    Lw = [pbar_x*pbar_y, -(1+pbar_x^2), pbar_y;
          (1+pbar_y^2), -pbar_x*pbar_y, -pbar_x];
    omega_cam = R_b2c * omega;
    
    pbar_next = pbar + (Lv * v_cam + Lw * omega_cam) * dt;
end
