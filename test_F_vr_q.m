function test_F_vr_q()
% Test the F_vr_q Jacobian specifically
% Investigate why analytical formulas don't match numerical derivatives

clear; clc;
fprintf('=== F_vr_q Jacobian Verification ===\n\n');

%% Check if quat2rotm normalizes internally
fprintf('=== Checking quat2rotm behavior ===\n');
q_scaled = [2; 0; 0; 0];  % Non-unit quaternion (magnitude 2)
R_scaled = quat2rotm(q_scaled');
R_unit = quat2rotm([1;0;0;0]');
normalizes = max(abs(R_scaled(:) - R_unit(:))) < 1e-10;
fprintf('quat2rotm normalizes internally: %s\n\n', mat2str(normalizes));

if normalizes
    fprintf('⚠️ IMPORTANT: quat2rotm NORMALIZES the quaternion!\n');
    fprintf('   This means ∂R/∂qi ≠ simple formula!\n');
    fprintf('   Must use chain rule: ∂R/∂qi = ∂R/∂q_normalized * ∂q_normalized/∂qi\n\n');
end

%% Test with identity quaternion
fprintf('=== TEST 1: Identity Quaternion ===\n');
q = [1; 0; 0; 0];
a = [1; 2; 3];
epsilon = 1e-8;

F_numerical = zeros(3, 4);
for i = 1:4
    q_plus = q; q_plus(i) = q_plus(i) + epsilon;
    q_minus = q; q_minus(i) = q_minus(i) - epsilon;
    
    R_plus = quat2rotm(q_plus');
    R_minus = quat2rotm(q_minus');
    
    F_numerical(:, i) = (R_plus * a - R_minus * a) / (2 * epsilon);
end

fprintf('Numerical ∂(R*a)/∂q at identity:\n');
disp(F_numerical);
fprintf('Notice: ∂/∂q0 (column 1) is NOT zero due to normalization effect!\n\n');

%% Correct formula accounting for normalization
fprintf('=== Correct Formula (with normalization) ===\n');
fprintf('Since quat2rotm(q) = quat2rotm(q/||q||), we need:\n');
fprintf('  ∂R(q)/∂qi = ∂R/∂q̂ * ∂q̂/∂qi  where q̂ = q/||q||\n\n');

% For unit quaternion q, perturbation by epsilon in direction i:
% q + eps*e_i => (q + eps*e_i) / ||q + eps*e_i||
%             ≈ (q + eps*e_i) * (1 - eps*q_i/||q||²)  (first order)
%             ≈ q + eps*(e_i - q*q_i/||q||²)
%             = q + eps*(e_i - q_i*q)  for unit q

% So the effective perturbation direction is (e_i - q_i*q), not e_i

%% Test at general point
fprintf('=== TEST 2: General Quaternion ===\n');
q = [0.9; 0.2; -0.3; 0.1];
q = q / norm(q);
a = [0.5; -0.3; 9.7];
dt = 0.001;

q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

% Numerical derivative
F_numerical = zeros(3, 4);
for i = 1:4
    q_plus = q; q_plus(i) = q_plus(i) + epsilon;
    q_minus = q; q_minus(i) = q_minus(i) - epsilon;
    
    R_plus = quat2rotm(q_plus');
    R_minus = quat2rotm(q_minus');
    
    F_numerical(:, i) = (R_plus * a - R_minus * a) / (2 * epsilon) * dt;
end

fprintf('Numerical ∂(R*a)/∂q * dt:\n');
disp(F_numerical);

%% Build correct Jacobian with normalization
% For unit quaternion, when we perturb qi, the normalized quaternion changes as:
% ∂q̂/∂qi = (I - q*q^T) * e_i = e_i - q_i*q

% The Jacobian of R(q̂)*a with respect to q is:
% ∂(R*a)/∂q = sum over j of (∂(R*a)/∂q̂_j) * (∂q̂_j/∂q_i)
%           = [∂(R*a)/∂q̂] * [I - q*q^T]

% First compute ∂(R*a)/∂q̂ (derivative assuming we control normalized q directly)
% This is the "raw" derivative without normalization

% ∂R/∂q0 (for unit quaternion, from R elements)
dR_dq0 = 2*[0, -q3, q2; q3, 0, -q1; -q2, q1, 0];
% ∂R/∂q1
dR_dq1 = 2*[0, q2, q3; q2, -2*q1, -q0; q3, q0, -2*q1];
% ∂R/∂q2
dR_dq2 = 2*[-2*q2, q1, q0; q1, 0, q3; -q0, q3, -2*q2];
% ∂R/∂q3
dR_dq3 = 2*[-2*q3, -q0, q1; q0, -2*q3, q2; q1, q2, 0];

% Raw Jacobian (if q were truly free)
J_raw = [dR_dq0*a, dR_dq1*a, dR_dq2*a, dR_dq3*a];

% Apply normalization correction: J_corrected = J_raw * (I - q*q')
I4 = eye(4);
normalization_jacobian = I4 - q*q';
J_corrected = J_raw * normalization_jacobian * dt;

fprintf('Corrected formula (accounting for normalization):\n');
disp(J_corrected);

fprintf('=== Errors ===\n');
fprintf('Raw formula vs Numerical:       max error = %.2e\n', max(abs(J_raw(:)*dt - F_numerical(:))));
fprintf('Corrected formula vs Numerical: max error = %.2e\n', max(abs(J_corrected(:) - F_numerical(:))));

if max(abs(J_corrected(:) - F_numerical(:))) < 1e-6
    fprintf('\n✅ CORRECTED FORMULA MATCHES!\n');
end

end
