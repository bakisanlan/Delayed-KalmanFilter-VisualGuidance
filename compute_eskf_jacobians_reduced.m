function [Fc, Gc, Fd, Gd] = compute_eskf_jacobians_reduced(x_nominal, omega_m, dt, R_b2c, R_b2e, p_i, v_i)
% COMPUTE_ESKF_JACOBIANS Compute Error-State Kalman Filter Jacobians
%
% Error state vector (20 states):
%   δx = [δpt(3), δvt(3), δpbar(2)]
%
% Nominal state vector (21 states):  
%   x = [pt(3), vt(3), pbar(2)]
%
% Inputs:
%   x_nominal - Nominal state vector (8x1)
%   omega_m   - Measured angular velocity (3x1)
%   a_m       - Measured body acceleration (3x1)
%   dt        - Time step
%   R_b2c     - Rotation from body to camera frame (3x3)
%
% Outputs:
%   Fc - Continuous-time error state Jacobian (8x8)
%   Gc - Continuous-time noise Jacobian (8x3)
%   Fd - Discrete-time state transition matrix (8x8)
%   Gd - Discrete-time noise Jacobian (8x3)

    %% ==================== Extract Nominal States ====================
    % State indices for nominal state
    idx_pt   = 1:3;
    idx_vt   = 4:6;
    idx_pbar = 7:8;
    
    p_t   = x_nominal(idx_pt);
    v_t   = x_nominal(idx_vt);
    pbar  = x_nominal(idx_pbar);
 
    pbar_x = pbar(1);
    pbar_y = pbar(2);
    
    %% ==================== Error State Indices ====================
    % Error state: δx = [δpt(3), δvt(3), δpbar(2)]
    idx_dpt    = 1:3;
    idx_dvt    = 4:6;
    idx_dpbar  = 7:8;
    
    %% ==================== Rotation Matrices ====================
    R_e2b = R_b2e';                     % Earth to Body
    % R_c2e = R_b2e * R_b2c';           % Camera to Earth
    % R_e2c = R_c2e';                   % Earth to Camera (= R_b2c * R_e2b)
    
    %% ==================== Measured IMU=======
    omega  = omega_m;          % angular velocity
    % a_body = a_m;              % body acceleration
    
    %% ==================== Camera Frame Quantities ====================
    % p_c = R_b2c * R_e2b * (-p_r)
    p_c = R_b2c * R_e2b * (-(p_i - p_t));
    p_c_z = max(p_c(3), 0.1);         % Depth (prevent division by zero)
    
    % v_c = R_b2c * R_e2b * v_r
    v_c = R_b2c * R_e2b * (v_i - v_t);
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
    
    %% ==================== Build Continuous-Time Fc Matrix (20x20) ====================
    Fc = zeros(8, 8);
   
    % --- Row 1-3: δpt_dot = δvt ---
    Fc(idx_dpt, idx_dvt) = eye(3);
    
    % --- Row 4-6: δvr_dot = 0 + noise ---
    
    % --- Row 7-8: δpbar_dot ---
    % δpbar_dot = A_pbar*δpbar + A_vc*δvc + A_pc_z*δpc_z
    %
    % Where (from add.m):
    %   δvc   = -R_b2c*R_e2b*δvt
    %   δpc_z =  e3*R_b2c*R_e2b*δpt
    
    e3 = [0; 0; 1];
    
    % δvc contribution
    dvc_ddvt    = -R_b2c * R_e2b;                % ∂δvc/∂δvr
    
    % δpc_z contribution  
    % pc = R_b2c * R_e2b * (-p_r),
    dpcz_ddpt    = e3' * R_b2c * R_e2b;              % 1x3
    
    % Assemble δpbar Jacobians
    % ∂δpbar_dot/∂δpt = A_pc_z * dpcz_ddpt
    Fc(idx_dpbar, idx_dpt) = A_pc_z * dpcz_ddpt;
    
    % ∂δpbar_dot/∂δvt = Lv * dvc_ddvt
    Fc(idx_dpbar, idx_dvt) = Lv * dvc_ddvt;
    
    % ∂δpbar_dot/∂δpbar = A_pbar
    Fc(idx_dpbar, idx_dpbar) = A_pbar;
    

    %% ==================== Build Continuous-Time Gc Matrix (20x15) ====================
    % Noise vector: n = [vn(3)]
    %   vn = target velocity noise

    Gc = zeros(8, 3);
    
    % δvt_dot affected by accel noise: vn
    Gc(idx_dvt, 1:3) = eye(3);
    
    % % δpbar_dot affected by gyro noise through δwc = -R_b2c*wn, assume δwc = -R_b2c*wn 
    % Gc(idx_dpbar, 1:3) = Lw * (-R_b2c);
    % % Gc(idx_dpbar, 1:3) = Lw * eye(3);
    
    %% ==================== Discretization ====================
    % Method 1: First-order approximation
    %   Fd ≈ I + Fc*dt
    %   Gd ≈ Gc*dt
    
    % Method 2: Matrix exponential (more accurate for attitude)
    %   Fd = expm(Fc*dt)
    
  
    % --- Build Fd (8x8) using matrix exponential for accuracy ---
    % First-order (Fd = I + Fc*dt) compounds errors badly during re-propagation.
    % expm is exact for the linearised system and avoids covariance blowup.
    Fd = expm(Fc * dt);
    
    % --- Gd (8x3) ---
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
