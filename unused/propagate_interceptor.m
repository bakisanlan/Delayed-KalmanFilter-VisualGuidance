function [p_int, v_int, q, omega, a_body] = propagate_interceptor(p_int, v_int, q, t, dt, g, e3)
% PROPAGATE_INTERCEPTOR Propagate interceptor dynamics for one time step
%
% Inputs:
%   p_int  - Current interceptor position (3x1)
%   v_int  - Current interceptor velocity (3x1)
%   q      - Current quaternion [q0; q1; q2; q3] (4x1)
%   t      - Current time (scalar)
%   dt     - Time step (scalar)
%   g      - Gravity constant (scalar)
%   e3     - Unit vector [0; 0; 1] (3x1)
%
% Outputs:
%   p_int  - Updated interceptor position (3x1)
%   v_int  - Updated interceptor velocity (3x1)
%   q      - Updated quaternion (4x1)
%   omega  - True angular velocity (3x1) - for IMU simulation
%   a_body - True body acceleration (3x1) - for IMU simulation

    % True angular velocity (small oscillations for realism)
    % Modify this for different interceptor maneuvers
    omega = 0.001 * sin(0.5 * t) * 0*[0; 0.5; 0];
    
    % Get current rotation matrix
    R_b2e = quat2rotm(q');
    
    % True specific force (gravity compensation + maneuver acceleration)
    % Modify this for different interceptor accelerations
    a_body = R_b2e' * (-g * e3) + sin(t) *  [0; 31; 0];
    
    % Acceleration in world frame
    a_world = R_b2e * a_body + g * e3;
    
    % Propagate quaternion
    omega_quat = [0; omega];
    dq = 0.5 * quatmultiply(q', omega_quat')';
    q = q + dq * dt;
    q = q / norm(q);
    
    % Propagate velocity and position (trapezoidal integration)
    v_int_new = v_int + a_world * dt;
    p_int = p_int + 0.5 * (v_int + v_int_new) * dt;
    v_int = v_int_new;
end
