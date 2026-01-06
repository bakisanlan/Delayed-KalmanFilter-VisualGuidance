function [Fc, Gc, Fd, Gd] = compute_eskf_jacobians(x_nominal, omega_m, a_m, dt, R_b2c)
% COMPUTE_ESKF_JACOBIANS Compute Error-State Kalman Filter Jacobians
%
% Error state vector (17 states):
%   δx = [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyro(3), δbacc(3)]
%
% Nominal state vector (18 states):  
%   x = [q(4), pr(3), vr(3), pbar(2), bgyro(3), bacc(3)]
%
% Inputs:
%   x_nominal - Nominal state vector (18x1)
%   omega_m   - Measured angular velocity (3x1)
%   a_m       - Measured body acceleration (3x1)
%   dt        - Time step
%   R_b2c     - Rotation from body to camera frame (3x3)
%
% Outputs:
%   Fc - Continuous-time error state Jacobian (17x17)
%   Gc - Continuous-time noise Jacobian (17x12)
%   Fd - Discrete-time state transition matrix (17x17)
%   Gd - Discrete-time noise Jacobian (17x12)

    %% ==================== Extract Nominal States ====================
    % State indices for nominal state
    idx_q    = 1:4;
    idx_pr   = 5:7;
    idx_vr   = 8:10;
    idx_pbar = 11:12;
    idx_bgyr = 13:15;
    idx_bacc = 16:18;
    
    q     = x_nominal(idx_q);
    p_r   = x_nominal(idx_pr);
    v_r   = x_nominal(idx_vr);
    pbar  = x_nominal(idx_pbar);
    b_gyr = x_nominal(idx_bgyr);
    b_acc = x_nominal(idx_bacc);
    
    pbar_x = pbar(1);
    pbar_y = pbar(2);
    
    %% ==================== Error State Indices ====================
    % Error state: δx = [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyro(3), δbacc(3)]
    idx_dtheta = 1:3;
    idx_dpr    = 4:6;
    idx_dvr    = 7:9;
    idx_dpbar  = 10:11;
    idx_dbgyr  = 12:14;
    idx_dbacc  = 15:17;
    
    %% ==================== Rotation Matrices ====================
    R_b2e = quat2rotm(q');           % Body to Earth
    R_e2b = R_b2e';                   % Earth to Body
    % R_c2e = R_b2e * R_b2c';           % Camera to Earth
    % R_e2c = R_c2e';                   % Earth to Camera (= R_b2c * R_e2b)
    
    %% ==================== Corrected IMU using nominal bias=======
    omega  = omega_m - b_gyr;          % Corrected angular velocity
    a_body = a_m - b_acc;              % Corrected body acceleration
    
    %% ==================== Camera Frame Quantities ====================
    % p_c = R_b2c * R_e2b * (-p_r)
    p_c = R_b2c * R_e2b * (-p_r);
    p_c_z = max(p_c(3), 0.1);         % Depth (prevent division by zero)
    
    % v_c = R_b2c * R_e2b * v_r
    v_c = R_b2c * R_e2b * v_r;
    v_c_x = v_c(1); v_c_y = v_c(2); v_c_z = v_c(3);
    
    % omega_c = R_b2c * omega
    omega_c = R_b2c * omega;
    w_c_x = omega_c(1); w_c_y = omega_c(2); w_c_z = omega_c(3);
    
    %% ==================== IBVS Jacobian Submatrices ====================
    % A_pbar: ∂(pbar_dot)/∂pbar
    A_pbar = [v_c_z/p_c_z + pbar_y*w_c_x - 2*pbar_x*w_c_y,  pbar_x*w_c_x + w_c_z;
              -pbar_y*w_c_y - w_c_z,                        v_c_z/p_c_z + 2*pbar_y*w_c_x - pbar_x*w_c_y];
    
    % A_vc: ∂(pbar_dot)/∂v_c (Lv matrix)
    Lv = [-1/p_c_z,     0,        pbar_x/p_c_z;
           0,          -1/p_c_z,  pbar_y/p_c_z];
    
    % A_wc: ∂(pbar_dot)/∂omega_c (Lw matrix)
    Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;
          (1+pbar_y^2),     -pbar_x*pbar_y, -pbar_x];
    
    % A_pc_z: ∂(pbar_dot)/∂p_c_z
    A_pc_z = [(v_c_x - pbar_x*v_c_z) / p_c_z^2;
              (v_c_y - pbar_y*v_c_z) / p_c_z^2];
    
    %% ==================== Build Continuous-Time Fc Matrix (17x17) ====================
    Fc = zeros(17, 17);
    
    % --- Row 1-3: δθ_dot = -skew(omega)*δθ - δbgyro ---
    Fc(idx_dtheta, idx_dtheta) = -skew(omega);
    Fc(idx_dtheta, idx_dbgyr)  = -eye(3);
    
    % --- Row 4-6: δpr_dot = δvr ---
    Fc(idx_dpr, idx_dvr) = eye(3);
    
    % --- Row 7-9: δvr_dot = -R*skew(a_body)*δθ - R*δbacc ---
    Fc(idx_dvr, idx_dtheta) = -R_b2e * skew(a_body);
    Fc(idx_dvr, idx_dbacc)  = -R_b2e;
    
    % --- Row 10-11: δpbar_dot ---
    % δpbar_dot = A_pbar*δpbar + A_vc*δvc + A_wc*δwc + A_pc_z*δpc_z
    %
    % Where (from errorstate.m):
    %   δvc   = R_b2c*R_e2b*δvr + R_b2c*skew(R_e2b*v_r)*δθ
    %   δwc   = -R_b2c*δbgyro
    %   δpc_z = -e3*R_b2c*R_e2b*δpr - e3*R_b2c*skew(R_e2b*p_r)*δθ
    
    e3 = [0; 0; 1];
    
    % δvc contribution
    dvc_ddtheta = R_b2c * skew(R_e2b * v_r);    % ∂δvc/∂δθ
    dvc_ddvr    = R_b2c * R_e2b;                % ∂δvc/∂δvr
    
    % δwc contribution
    dwc_ddbgyr = -R_b2c;                        % ∂δwc/∂δbgyro
    
    % δpc_z contribution  
    % pc = R_b2c * R_e2b * (-p_r), so ∂pc/∂δθ involves skew(R_e2b * (-p_r))
    dpcz_ddtheta = -e3' * R_b2c * skew(R_e2b * p_r);  % 1x3
    dpcz_ddpr    = -e3' * R_b2c * R_e2b;              % 1x3
    
    % Assemble δpbar Jacobians
    % ∂δpbar_dot/∂δθ = Lv * dvc_ddtheta + A_pc_z * dpcz_ddtheta
    Fc(idx_dpbar, idx_dtheta) = Lv * dvc_ddtheta + A_pc_z * dpcz_ddtheta;
    
    % ∂δpbar_dot/∂δpr = A_pc_z * dpcz_ddpr
    Fc(idx_dpbar, idx_dpr) = A_pc_z * dpcz_ddpr;
    
    % ∂δpbar_dot/∂δvr = Lv * dvc_ddvr
    Fc(idx_dpbar, idx_dvr) = Lv * dvc_ddvr;
    
    % ∂δpbar_dot/∂δpbar = A_pbar
    Fc(idx_dpbar, idx_dpbar) = A_pbar;
    
    % ∂δpbar_dot/∂δbgyro = Lw * dwc_ddbgyr
    Fc(idx_dpbar, idx_dbgyr) = Lw * dwc_ddbgyr;
    
    % --- Row 12-14: δbgyro_dot = 0 (random walk handled by noise) ---
    % Fc(idx_dbgyr, :) = 0  (already zero)
    
    % --- Row 15-17: δbacc_dot = 0 (random walk handled by noise) ---
    % Fc(idx_dbacc, :) = 0  (already zero)
    
    %% ==================== Build Continuous-Time Gc Matrix (17x12) ====================
    % Noise vector: n = [wn(3), an(3), ww(3), aw(3)]
    %   wn = gyro measurement noise
    %   an = accel measurement noise
    %   ww = gyro bias random walk
    %   aw = accel bias random walk
    % (-) is not logical, because they are white noise around 0

    Gc = zeros(17, 12);
    
    % δθ_dot affected by gyro noise: -wn 
    Gc(idx_dtheta, 1:3) = -eye(3);
    % Gc(idx_dtheta, 1:3) = eye(3);

    
    % δvr_dot affected by accel noise: -R*an, assume R*an = an 
    Gc(idx_dvr, 4:6) = -R_b2e;
    % Gc(idx_dvr, 4:6) = eye(3);
    
    % δpbar_dot affected by gyro noise through δwc = -R_b2c*wn, assume δwc = -R_b2c*wn 
    Gc(idx_dpbar, 1:3) = Lw * (-R_b2c);
    % Gc(idx_dpbar, 1:3) = Lw * eye(3);

    
    % δbgyro_dot affected by bias random walk: ww
    Gc(idx_dbgyr, 7:9) = eye(3);
    
    % δbacc_dot affected by bias random walk: aw
    Gc(idx_dbacc, 10:12) = eye(3);
    
    %% ==================== Discretization ====================
    % Method 1: First-order approximation
    %   Fd ≈ I + Fc*dt
    %   Gd ≈ Gc*dt
    
    % Method 2: Matrix exponential (more accurate for attitude)
    %   Fd = expm(Fc*dt)
    
    % For attitude subsystem, use exact exponential map
    % For other states, use first-order approximation
    
    % --- Exact discretization for attitude ---
    % δθ_dot = -skew(omega)*δθ - δbgyro
    % The homogeneous solution: δθ(t+dt) = exp(-skew(omega)*dt) * δθ(t)
    % Using Rodrigues formula: exp(-skew(omega)*dt) = exp_rot(-omega*dt)
    
    omega_dt = omega * dt;
    Phi_theta = exp_rot(-omega_dt);  % 3x3 rotation  % NOTE: CHECK THAT sign
    
    % --- Build Fd (17x17) ---
    Fd = eye(17) + Fc * dt;
    
    % Replace attitude block with exact exponential
    Fd(idx_dtheta, idx_dtheta) = Phi_theta;
    
    % Correct the attitude-to-bias coupling
    % δθ(k+1) = Phi_theta * δθ(k) + Integral term for δbgyro
    % For small dt: Integral ≈ -I * dt (first order) or more accurate:
    % Using: ∫₀^dt exp(-skew(ω)τ) dτ ≈ (I - Phi_theta) * inv(skew(ω)) for ω≠0
    % For simplicity, use first-order approximation:
    Fd(idx_dtheta, idx_dbgyr) = -eye(3) * dt;
    
    % --- Gd (17x12) ---
    Gd = Gc * dt;
    
    % For more accuracy on attitude noise, could use integral of Phi_theta
    % but first-order is usually sufficient
    
end

%% ==================== Helper Functions ====================

function S = skew(v)
    % Skew-symmetric matrix from 3-vector
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

function R = exp_rot(d_rot)
    % Rotation matrix from axis-angle using Rodrigues formula
    % R = I*cos(θ) + sin(θ)*skew(u) + (1-cos(θ))*u*u'
    % where θ = ||d_rot|| and u = d_rot/θ
    
    d_rot = reshape(d_rot, [], 1);
    theta = norm(d_rot);
    
    if theta < 1e-10
        R = eye(3);
        return;
    end
    
    u = d_rot / theta;
    u_skew = skew(u);
    
    R = eye(3)*cos(theta) + sin(theta)*u_skew + (1-cos(theta))*(u*u');
end
