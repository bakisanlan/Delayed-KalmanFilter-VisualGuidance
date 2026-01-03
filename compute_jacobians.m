function [F, G] = compute_jacobians(x, omega_meas, a_body_meas, dt, R_b2c,   ...
                                    idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc)
% COMPUTE_JACOBIANS Compute state transition and process noise Jacobians
%
% Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
% by Kun Yang et al.
%
% Inputs:
%   x        - State vector (18x1)
%   omega    - Corrected angular velocity (3x1)
%   a_body   - Corrected body acceleration (3x1)
%   dt       - Time step
%   R_b2c    - Rotation from body to camera frame (3x3)
%   R_b2e    - Rotation from body to earth frame (3x3)
%   p_zc     - Depth in camera frame
%   v_r      - Relative velocity (3x1)
%   pbar     - Normalized image coordinates (2x1)
%   idx_*    - State indices
%
% Outputs:
%   F - State transition Jacobian (18x18)
%   G - Process noise Jacobian (18x6)

    % Extract states and p_zc
    q = x(idx_q);
    p_r = x(idx_pr);
    v_r = x(idx_vr);
    pbar = x(idx_pbar);
    bgyr = x(idx_bgyr);
    bacc = x(idx_bacc);
    R_b2e = quat2rotm(q');

    e3 = [0 0 1];
    p_zc = e3 * R_b2c * R_b2e' * (-p_r);
    p_zc = max(p_zc, 0.1);

    % Bias removal from IMU measurement
    omega  = omega_meas  - bgyr;
    a_body = a_body_meas - bacc;

    wx = omega(1); wy = omega(2); wz = omega(3);

    % Extract quaternion components
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    pbar_x = pbar(1); pbar_y = pbar(2);
    
    % Initialize F as identity
    F = eye(18);
    
    %% QUATERNION PART
    % F_q_q: Quaternion to quaternion (M(delta_q))
    F(1:4, 1:4) = [1,        -wx*dt/2, -wy*dt/2, -wz*dt/2;
                   wx*dt/2,   1,        wz*dt/2, -wy*dt/2;
                   wy*dt/2,  -wz*dt/2,  1,        wx*dt/2;
                   wz*dt/2,   wy*dt/2, -wx*dt/2,  1];
    
    % F_q_bgyr: Quaternion to gyro bias
    F_q_bgyr = [q1/2,   q2/2,   q3/2;
               -q0/2,   q3/2,  -q2/2;
               -q3/2,  -q0/2,   q1/2;
                q2/2,  -q1/2,  -q0/2] * dt;
    F(1:4, idx_bgyr) = F_q_bgyr;
    
    %% F_pr_vr: Position to velocity
    F(idx_pr, idx_vr) = eye(3) * dt;
    
    %% VELOCITY PART (F_vr_q)  % NOTE: When I use paper formula filter diverge
    % F_vr_q: Velocity to quaternion
    
    % ========== OLD PAPER FORMULA (commented out) ==========
    % M1, M2, M3 matrices from paper (does NOT account for quat2rotm normalization)
    % M1 = [q0, -q3,  q2;
    %       q1,  q2,  q3;
    %      -q2,  q1,  q0;
    %      -q3, -q0,  q1];
    % M2 = [q3,  q0, -q1;
    %       q2, -q1, -q0;
    %       q1,  q2,  q3;
    %       q0, -q3,  q2];
    % M3 = [-q2,  q1,  q0;
    %        q3,  q0, -q1;
    %       -q0,  q3, -q2;
    %        q1,  q2,  q3];
    % F_vr_q = 2 * [M1 * a_body, M2 * a_body, M3 * a_body]' * dt;
    % ========== END OLD FORMULA ==========
    
    % ========== NEW CORRECTED FORMULA ==========
    % IMPORTANT: MATLAB's quat2rotm normalizes internally, so we must apply
    % chain rule correction: J_corrected = J_raw * (I - q*q')
    
    % Raw partial derivatives of R*a with respect to each quaternion component
    % % ∂R/∂q0 (skew-symmetric part)
    dR_dq0 = 2 * [0, -q3, q2; q3, 0, -q1; -q2, q1, 0];
    % ∂R/∂q1
    dR_dq1 = 2 * [0, q2, q3; q2, -2*q1, -q0; q3, q0, -2*q1];
    % ∂R/∂q2
    dR_dq2 = 2 * [-2*q2, q1, q0; q1, 0, q3; -q0, q3, -2*q2];
    % ∂R/∂q3
    dR_dq3 = 2 * [-2*q3, -q0, q1; q0, -2*q3, q2; q1, q2, 0];
    % 
    % % Raw Jacobian (without normalization correction)
    J_raw = [dR_dq0*a_body, dR_dq1*a_body, dR_dq2*a_body, dR_dq3*a_body];
    % 
    % % Normalization correction: for unit quaternion, perturbation projects onto tangent space
    % % ∂q̂/∂q = I - q*q' (where q̂ = q/||q||)
    normalization_correction = eye(4) - q * q';
    % 
    % % Corrected F_vr_q
    F_vr_q = J_raw * normalization_correction * dt;
    F(idx_vr, idx_q) = F_vr_q;
    % ========== END NEW FORMULA ==========
    
    % F_vr_bacc: Velocity to accelerometer bias
    F(idx_vr, idx_bacc) = -R_b2e * dt;
    
    %% PBAR PART %NOTE both paper and new version are working but I guess new is better
    % F_pbar_q: Image feature to quaternion
    
    % % ========== OLD PAPER FORMULA (commented out) ==========
    % % M4 and M5 matrices from paper (does NOT account for quat2rotm normalization)
    % M4 = [2*pbar_x*q0+2*q3,   2*pbar_x*q3-2*q0,  -2*pbar_x*q2-2*q1;
    %       2*pbar_x*q1-2*q2,   2*pbar_x*q2+2*q1,   2*pbar_x*q3-2*q0;
    %       2*pbar_x*q2-2*q1,   2*pbar_x*q1-2*q2,  -2*pbar_x*q0-2*q3;
    %       2*pbar_x*q3+2*q0,   2*pbar_x*q0+2*q3,   2*pbar_x*q1-2*q2];
    % M5 = [2*pbar_y*q0-2*q2,   2*pbar_y*q3+2*q1,  -2*pbar_y*q2-2*q0;
    %       2*pbar_y*q1-2*q3,   2*pbar_y*q2+2*q0,   2*pbar_y*q3+2*q1;
    %       2*pbar_y*q2-2*q0,   2*pbar_y*q1-2*q3,  -2*pbar_y*q0+2*q2;
    %       2*pbar_y*q3-2*q1,   2*pbar_y*q0-2*q2,   2*pbar_y*q1-2*q3];
    % F_pbar_q = (1/p_zc) * [M4 * v_r, M5 * v_r]' * dt;
    % % ========== END OLD FORMULA ==========
    
    % ========== USER'S ALTERNATIVE FORMULA (commented out) ==========
    % Alternative derivation using 3x3 M matrices for ∂R'/∂qi
    % M1 = [ q0, -q3,  q2;
    %        q3,  q0, -q1;
    %       -q2,  q1,  q0];
    % M2 = [ q1,  q2,  q3;
    %        q2, -q1, -q0;
    %        q3,  q0, -q1];
    % M3 = [-q2,  q1,  q0;
    %        q1,  q2,  q3;
    %       -q0,  q3, -q2];
    % M4 = [-q3, -q0,  q1;
    %        q0, -q3,  q2;
    %        q1,  q2,  q3];
    % 
    % p_r = x(idx_pr);
    % v_r = x(idx_vr);
    % e = [0 0 1];
    % pbar_m = [-1 0 pbar_x ; 0 -1 pbar_y];
    % denom = (e*R_b2c*R_b2e'*-p_r)^2;
    % m1 = pbar_m * R_b2c * R_b2e';
    % m2 = (1/p_zc) * pbar_m;
    % 
    % F_pbar_aux1_1 = (-e * (R_b2c*M1*-p_r) / denom) * m1;
    % F_pbar_aux1_2 = (-e * (R_b2c*M2*-p_r) / denom) * m1;
    % F_pbar_aux1_3 = (-e * (R_b2c*M3*-p_r) / denom) * m1;
    % F_pbar_aux1_4 = (-e * (R_b2c*M4*-p_r) / denom) * m1;
    % 
    % F_pbar_aux2_1 = m2 * M1;
    % F_pbar_aux2_2 = m2 * M2;
    % F_pbar_aux2_3 = m2 * M3;
    % F_pbar_aux2_4 = m2 * M4;
    % 
    % F_pbar_aux_total1 = (F_pbar_aux1_1 + F_pbar_aux2_1)*v_r;
    % F_pbar_aux_total2 = (F_pbar_aux1_2 + F_pbar_aux2_2)*v_r;
    % F_pbar_aux_total3 = (F_pbar_aux1_3 + F_pbar_aux2_3)*v_r;
    % F_pbar_aux_total4 = (F_pbar_aux1_4 + F_pbar_aux2_4)*v_r;
    % F_pbar_q = [F_pbar_aux_total1 F_pbar_aux_total2 F_pbar_aux_total3 F_pbar_aux_total4] * normalization_correction * dt;
    % ========== END USER'S ALTERNATIVE FORMULA ==========
    
    % ========== NEW CORRECTED FORMULA ==========
    % Uses same normalization correction as F_vr_q
    % ALSO includes p_zc dependency on q (p_zc = [R_b2c * R' * (-p_r)]_z)
    % The pbar dynamics depend on R_b2e through v_cam = R_b2c * R_b2e' * v_r
    % Full Jacobian: ∂pbar/∂q = ∂Lv/∂p_zc * ∂p_zc/∂q * v_cam + Lv * ∂v_cam/∂q

    % Lv matrix for image feature dynamics
    Lv_pbar = [-1/p_zc, 0, pbar_x/p_zc;
                0, -1/p_zc, pbar_y/p_zc];

    v_cam = R_b2c * R_b2e' * v_r;

    % --- TERM 1: ∂Lv/∂p_zc * ∂p_zc/∂q * v_cam ---
    % ∂Lv/∂p_zc
    dLv_dpzc = [1/p_zc^2, 0, -pbar_x/p_zc^2;
                0, 1/p_zc^2, -pbar_y/p_zc^2];

    % ∂p_zc/∂q: p_zc = [R_b2c * R' * (-p_r)]_z
    % ∂p_zc/∂qi = e_z' * R_b2c * (∂R'/∂qi) * (-p_r)
    dRt_pr_dq0 = dR_dq0' * (-p_r);
    dRt_pr_dq1 = dR_dq1' * (-p_r);
    dRt_pr_dq2 = dR_dq2' * (-p_r);
    dRt_pr_dq3 = dR_dq3' * (-p_r);

    dpzc_dq_raw = e3 * R_b2c * [dRt_pr_dq0, dRt_pr_dq1, dRt_pr_dq2, dRt_pr_dq3];
    dpzc_dq = dpzc_dq_raw * normalization_correction;  % 1x4

    term1 = dLv_dpzc * v_cam * dpzc_dq;  % (2x3) * (3x1) * (1x4) = 2x4

    % --- TERM 2: Lv * ∂v_cam/∂q ---
    % ∂v_cam/∂q = R_b2c * ∂(R'*v_r)/∂q
    dRt_v_dq0 = dR_dq0' * v_r;
    dRt_v_dq1 = dR_dq1' * v_r;
    dRt_v_dq2 = dR_dq2' * v_r;
    dRt_v_dq3 = dR_dq3' * v_r;

    dv_cam_dq_raw = R_b2c * [dRt_v_dq0, dRt_v_dq1, dRt_v_dq2, dRt_v_dq3];
    dv_cam_dq = dv_cam_dq_raw * normalization_correction;  % 3x4

    term2 = Lv_pbar * dv_cam_dq;  % 2x4

    % --- FULL F_pbar_q ---
    F_pbar_q = (term1 + term2) * dt;
    F(idx_pbar, idx_q) = F_pbar_q;
    % ========== END NEW FORMULA ==========

    % F_pbar_pr: Image feature to position (p_zc depends on p_r)
    d_pbar_pzc = (1/p_zc)^2 * [1 0 -pbar_x; 0 1 -pbar_y] * R_b2c * R_b2e' * v_r * dt;
    d_pc_pr    = -R_b2c * R_b2e';
    d_pcz_pr   = d_pc_pr(3,:);
    d_pbar_pr  = d_pbar_pzc * d_pcz_pr;
    F(idx_pbar, idx_pr) = d_pbar_pr;

    % F_pbar_vr: Image feature to velocity
    Lv = [-1/p_zc,     0,    pbar_x/p_zc;
           0,     -1/p_zc,   pbar_y/p_zc];
    F(idx_pbar, idx_vr) = Lv * R_b2c * R_b2e' * dt;
    
    % F_pbar_pbar: Image feature to image feature
    v_cam = R_b2c * R_b2e' * v_r;
    omega_cam = R_b2c * omega;
    vz_c = v_cam(3);
    ox_c = omega_cam(1); oy_c = omega_cam(2); oz_c = omega_cam(3);
    
    F_pbar_pbar = eye(2) + [vz_c/p_zc + pbar_y*ox_c - 2*pbar_x*oy_c,  pbar_x*ox_c + oz_c;
                            -pbar_y*oy_c - oz_c,  vz_c/p_zc + 2*pbar_y*ox_c - pbar_x*oy_c] * dt;
    F(idx_pbar, idx_pbar) = F_pbar_pbar;
    
    % F_pbar_bgyr: Image feature to gyro bias
    Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;
          (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
    F(idx_pbar, idx_bgyr) = -Lw * R_b2c * dt;
    
    %% Process noise Jacobian G (18x6)
    G = zeros(18, 6);
    
    % G for quaternion (from gyro bias noise)
    G(1:4, 1:3) = F_q_bgyr;
    
    % G for velocity (from accel bias noise)
    G(idx_vr, 4:6) = -R_b2e * dt;
    
    % G for image feature (from gyro bias noise)
    G(idx_pbar, 1:3) = F(idx_pbar, idx_bgyr);
end

function S = skew(v)
%SKEW 3x3 skew-symmetric matrix for cross product.
% S*x = v x x
    v = v(:);
    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0  ];
end