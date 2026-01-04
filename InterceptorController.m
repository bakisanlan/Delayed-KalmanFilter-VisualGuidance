classdef InterceptorController < handle
    % INTERCEPTORCONTROLLER Guidance controller for drone interception
    %
    % Generates body-frame control commands (angular velocity and linear acceleration)
    % to guide the interceptor toward the target based on ESKF state estimates.
    
    properties
        Kp_att     % Proportional gain for attitude control
        Kp_acc     % Proportional gain for position control (approach)
        max_omega  % Maximum angular velocity limit [rad/s]
        max_acc    % Maximum acceleration limit [m/s^2]
        
        g = 9.81;  % Gravity [m/s^2]
        e3 = [0;0;1];
    end
    
    methods
        function obj = InterceptorController(params)
            % Constructor
            % params.Kp_att: Attitude gain (default: 5)
            % params.Kp_acc: Acceleration gain (default: 2)
            
            if nargin < 1; params = struct(); end
            
            if isfield(params, 'Kp_att'); obj.Kp_att = params.Kp_att; else; obj.Kp_att = 5; end
            if isfield(params, 'Kp_acc'); obj.Kp_acc = params.Kp_acc; else; obj.Kp_acc = 2; end
            if isfield(params, 'max_omega'); obj.max_omega = params.max_omega; else; obj.max_omega = 5; end
            if isfield(params, 'max_acc'); obj.max_acc = params.max_acc; else; obj.max_acc = 20; end
        end
        
        function [omega_cmd, a_cmd] = compute_control(obj, q_est, p_r_est, v_r_est)
            % COMPUTE_CONTROL Generate control commands based on estimated state
            %
            % Inputs:
            %   q_est   - Estimated quaternion [q0; q1; q2; q3] (scalar first)
            %   p_r_est - Estimated relative position (p_int - p_tgt) [m]
            %   v_r_est - Estimated relative velocity (v_int - v_tgt) [m/s]
            %
            % Outputs:
            %   omega_cmd - Commanded body angular velocity [rad/s]
            %   a_cmd     - Commanded body specific force [m/s^2] (includes gravity comp)
            
            % 1. LINE-OF-SIGHT (LOS) VECTOR
            % p_r_est = p_int - p_tgt
            % Vector FROM Interceptor TO Target in NED frame:
            los_vector_ned = -p_r_est; 
            dist = norm(los_vector_ned);
            
            if dist < 1e-3
                % Too close, keep current state or stop
                omega_cmd = zeros(3,1);
                a_cmd = zeros(3,1);
                return;
            end
            
            los_unit_ned = los_vector_ned / dist;
            
            % 2. ATTITUDE CONTROL (ALIGN BODY X-AXIS WITH LOS)
            % Current rotation matrix Body -> NED
            R_b2e = quat2rotm(q_est');
            
            % Transform LOS vector to Body frame
            % v_body = R_e2b * v_ned = R_b2e' * v_ned
            los_body = R_b2e' * los_unit_ned;
            
            % Desired body x-axis is [1; 0; 0]
            % We want los_body to be aligned with [1; 0; 0]
            % Rotation error vector (cross product)
            % The axis of rotation required to bring x_body to los_body
            % is cross(x_body, los_body)
            x_body = [1; 0; 0];
            
            % Cross product: x_body x los_body
            % = [0; -los_body(3); los_body(2)]
            err_vec = cross(x_body, los_body);
            
            % This error vector direction is the axis of rotation, 
            % magnitude is sin(theta). For small angles, approx theta.
            
            % Proportional control for angular velocity
            omega_cmd = obj.Kp_att * err_vec;
            
            % Add feedforward term? (Optional, if we knew target motion)
            % For now, just P-control.
            
            % Saturation
            if norm(omega_cmd) > obj.max_omega
                omega_cmd = obj.max_omega * omega_cmd / norm(omega_cmd);
            end
            
            % 3. ACCELERATION CONTROL (APPROACH TARGET)
            % Limit speed to 20 m/s
            v_max = 20; 
            
            % Desired velocity is along the LOS vector with magnitude v_max
            % v_r is defined as v_int - v_tgt.
            % We want v_int to move towards target relative to target.
            % So v_r_des should be v_max * los_unit_ned
            v_r_des = v_max * los_unit_ned;
            
            % Simple P-control on velocity
            % a_kinematic = Kp * (v_des - v_curr)
            a_kinematic_ned = obj.Kp_acc * (v_r_des - v_r_est);
            
            % 4. GRAVITY COMPENSATION
            % The IMU measures specific force (a_meas = a_kinematic - g).
            % We control the thrust/forces which result in specific force.
            % So the "commanded acceleration" to the drone dynamics (propagate_true_state)
            % corresponds to a_body_true in the previous code.
            % a_body_true = R_b2e' * (a_int_world - g*e3)
            % Wait, a_int_world IS a_kinematic_ned.
            
            % We want specific force command:
            % f_b = R_b2e' * (a_des_ned - g_ned)
            
            g_ned = [0; 0; obj.g];
            a_cmd = R_b2e' * (a_kinematic_ned - g_ned);
            
            % Saturation
            if norm(a_cmd) > obj.max_acc
                a_cmd = obj.max_acc * a_cmd / norm(a_cmd);
            end
            
        end
    end
end
