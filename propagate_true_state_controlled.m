function [x_true, p_int_new, v_int_new, p_tgt_new, v_tgt_new, omega_true, a_body_true] = ...
    propagate_true_state_controlled(x_true, p_int, v_int, p_tgt, v_tgt, ...
                         t, dt, g, e3, R_b2c, ...
                         omega_in, acc_in, ...
                         idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc)
% PROPAGATE_TRUE_STATE_CONTROLLED Propagate true state with CONTROL INPUTS
%
% Merges interceptor dynamics, target dynamics, and image feature dynamics
% into a single function that outputs the complete true state vector.
%
% Inputs:
%   x_true    - Current true state vector (18x1)
%   p_int     - Interceptor position (3x1)
%   v_int     - Interceptor velocity (3x1)
%   p_tgt     - Target position (3x1)
%   v_tgt     - Target velocity (3x1)
%   t         - Current time
%   dt        - Time step
%   g         - Gravity constant
%   e3        - Unit vector [0; 0; 1]
%   R_b2c     - Rotation from body to camera frame
%   omega_in  - Input angular velocity command [rad/s]
%   acc_in    - Input specific force command [m/s^2]
%   idx_*     - State indices
%
% Outputs:
%   x_true      - Updated true state vector (18x1)
%   p_int       - Updated interceptor position
%   v_int       - Updated interceptor velocity
%   q           - Updated quaternion
%   p_tgt       - Updated target position
%   v_tgt       - Updated target velocity
%   omega_true  - True angular velocity (for IMU simulation)
%   a_body_true - True body acceleration (for IMU simulation)

    % Extract current image features and biases
    q_int = x_true(idx_q);
    p_r = x_true(idx_pr);
    v_r = x_true(idx_vr);
    pbar = x_true(idx_pbar);
    b_gyr_true = x_true(idx_bgyr);
    b_acc_true = x_true(idx_bacc);
    
    %% ==================== INTERCEPTOR DYNAMICS ====================
    % True angular velocity (driven by controller)
    omega_true = omega_in;

    % Get current rotation matrix
    R_b2e = quat2rotm(q_int');
    
    % True specific force (driven by controller)
    a_body_true = acc_in;

    % Acceleration in world frame
    % a_world = R_b * a_body + g
    % But a_body_true is specific force = (a_kinematic - g_body)
    % So a_kinematic_body = a_body_true + g_body
    % a_kinematic_world = R_b * (a_body_true + R_b' * g) = R_b * a_body_true + g
    a_int_world = R_b2e * a_body_true + g * e3;
    
    % Propagate quaternion
    omega_quat = [0; omega_true];
    dq = 0.5 * quatmultiply(q_int', omega_quat')';
    q_int_new = q_int + dq * dt;
    q_int_new = q_int_new / norm(q_int_new); % Normalize quaternion
    
    % Propagate interceptor velocity and position (trapezoidal)
    v_int_new = v_int + a_int_world * dt;
    p_int_new = p_int + 0.5 * (v_int + v_int_new) * dt;
    
    %% ==================== TARGET DYNAMICS ====================
    % Target acceleration (modify for different target behaviors)
    a_tgt = 0.01*[1; 2; -3];  % Static target
    
    % Propagate target velocity and position
    v_tgt_new = v_tgt + a_tgt * dt;
    % v_tgt_new = 0.5*[-3 ; -4 ; 5];
    p_tgt_new = p_tgt + 0.5 * (v_tgt + v_tgt_new) * dt;

    %% ==================== RELATIVE DYNAMICS ====================
    p_r_new = p_int_new - p_tgt_new;
    v_r_new = v_int_new - v_tgt_new;
    
    %% ==================== IMAGE FEATURE DYNAMICS ====================
    % Same equations as in DelayedKalmanFilter.predictState (line 214)
    % Update rotation matrix with new quaternion
    R_b2e = quat2rotm(q_int');
    
    % Compute depth in camera frame
    p_r_cam = R_b2c * R_b2e' * (-p_r);
    p_zc = p_r_cam(3);
    % disp(p_zc)
    
    % Check if target is in front of camera
    if p_zc > 0.1 % Relaxed tolerance slightly
        pbar_x = pbar(1);
        pbar_y = pbar(2);
        
        % Linear velocity contribution (Lv matrix)
        Lv = [-1/p_zc,     0,    pbar_x/p_zc;
               0,     -1/p_zc,   pbar_y/p_zc];
        v_cam = R_b2c * R_b2e' * v_r;
        
        % Angular velocity contribution (Lw matrix)
        Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;
              (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
        omega_cam = R_b2c * omega_true;
        
        % Image feature dynamics
        pbar_dot = Lv * v_cam + Lw * omega_cam;
        pbar_new = pbar + pbar_dot * dt;
    else
        % warning('Target behind camera (p_zc = %.2f)', p_zc);
        pbar_new = [0;0];
    end
    
    %% ==================== ASSEMBLE TRUE STATE ====================
    x_true = [q_int_new; p_r_new; v_r_new; pbar_new; b_gyr_true; b_acc_true];
end
