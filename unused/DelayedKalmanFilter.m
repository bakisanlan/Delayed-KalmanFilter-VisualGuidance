classdef DelayedKalmanFilter < handle
    % DelayedKalmanFilter - Delayed Kalman Filter for Visual Guidance
    % Based on: "High-Speed Interception Multicopter Control by Image-Based Visual Servoing"
    % by Kun Yang et al.
    %
    % State vector x (18x1):
    % x = [q(4), p_r(3), v_r(3), p_bar(2), b_gyr(3), b_acc(3)]^T
    
    properties
        % State indices
        idx_q     = 1:4;      % Quaternion
        idx_pr    = 5:7;      % Relative position
        idx_vr    = 8:10;     % Relative velocity
        idx_pbar  = 11:12;    % Normalized image coordinates
        idx_bgyr  = 13:15;    % Gyroscope bias
        idx_bacc  = 16:18;    % Accelerometer bias
        
        % State and covariance
        x         % Current state estimate (18x1)
        P         % Current covariance matrix (18x18)
        
        % Physical constants
        g = 9.81;             % Gravity (m/s^2)
        e3 = [0; 0; 1];       % Unit vector in z-direction
        
        % Camera parameters
        R_b2c                 % Rotation from body to camera frame
        
        % Noise parameters
        Q                     % Process noise covariance (6x6)
        R_img                 % Image measurement noise covariance (2x2)
        R_radar               % RADAR measurement noise covariance (3x3)
        
        % Timing
        dt_imu                % IMU sample time
        
        % History buffers for delay handling
        history_length        % Number of steps to store
        x_history             % State history
        P_history             % Covariance history
        imu_history           % IMU measurement history
        time_history          % Time history
        
        % Measurement matrices
        H_img                 % Image measurement matrix (2x18)
        H_radar               % RADAR measurement matrix (3x18)
    end
    
    methods
        function obj = DelayedKalmanFilter(params)
            % Constructor
            % params: struct with initialization parameters
            
            % Set timing
            obj.dt_imu = params.dt_imu;
            
            % Set camera rotation
            obj.R_b2c = params.R_b2c;
            
            % Set noise covariances
            obj.Q = params.Q;
            obj.R_img = params.R_img;
            if isfield(params, 'R_radar')
                obj.R_radar = params.R_radar;
            else
                obj.R_radar = eye(3);  % Default
            end
            
            % Initialize state
            obj.x = params.x_init;
            obj.P = params.P_init;
            
            % Setup history buffers
            obj.history_length = max(params.delay_steps + 10, 15);
            obj.x_history = zeros(18, obj.history_length);
            obj.P_history = zeros(18, 18, obj.history_length);
            obj.imu_history = zeros(6, obj.history_length);
            obj.time_history = zeros(1, obj.history_length);
            
            % Initialize history with initial state
            for i = 1:obj.history_length
                obj.x_history(:, i) = obj.x;
                obj.P_history(:, :, i) = obj.P;
            end
            
            % Setup image measurement matrix (measures pbar)
            obj.H_img = zeros(2, 18);
            obj.H_img(1:2, obj.idx_pbar) = eye(2);
            
            % Setup RADAR measurement matrix (measures p_r directly)
            obj.H_radar = zeros(3, 18);
            obj.H_radar(1:3, obj.idx_pr) = eye(3);
        end
        
        function predict(obj, omega_meas, a_meas, t)
            % Prediction step using IMU measurements
            % omega_meas: measured angular velocity (3x1)
            % a_meas: measured acceleration (3x1)
            % t: current time
                        
            % Predict state
            x_pred = obj.predictState(obj.x, omega_meas, a_meas);
            
            % Compute Jacobians
            [F, G] = compute_jacobians(obj.x, omega_meas, a_meas, obj.dt_imu, obj.R_b2c, ...
                                       obj.idx_q,    obj.idx_pr,   obj.idx_vr, ...
                                       obj.idx_pbar, obj.idx_bgyr, obj.idx_bacc);
            
            % Predict covariance
            P_pred = F * obj.P * F' + G * obj.Q * G';
            
            % Update state and covariance
            obj.x = x_pred;
            obj.P = P_pred;
            
            % Normalize quaternion
            % obj.x(obj.idx_q) = obj.x(obj.idx_q) / norm(obj.x(obj.idx_q));
            
            % Update history buffers
            obj.updateHistory(omega_meas, a_meas, t);
        end
        
        function innovation = correct(obj, z_meas, delay_steps)
            % Correction step with delayed IMAGE measurement
            % z_meas: image measurement (2x1)
            % delay_steps: number of IMU steps the measurement is delayed
            %
            % Returns: innovation (2x1) or empty if correction not applied
            
            innovation = [];
            
            % Find the delayed state in history
            idx_delayed = obj.history_length - delay_steps;
            
            if idx_delayed < 1
                return;  % Not enough history
            end
            
            % Get prior state estimate from D steps ago
            x_prior = obj.x_history(:, idx_delayed);
            P_prior = obj.P_history(:, :, idx_delayed);
            
            % Kalman gain (using image measurement matrix)
            H = obj.H_img;
            R = obj.R_img;
            S = H * P_prior * H' + R;
            K = P_prior * H' / S;
            
            % Innovation
            z_pred = H * x_prior;
            innovation = z_meas - z_pred;
            
            % Correct the delayed state
            x_corrected = x_prior + K * innovation;
            % Joseph form for numerical stability
            P_corrected = (eye(18) - K * H) * P_prior * (eye(18) - K * H)' + K * R * K';

            % Normalize quaternion
            % x_corrected(obj.idx_q) = x_corrected(obj.idx_q) / norm(x_corrected(obj.idx_q));
            
            % Re-propagate from corrected delayed state to current time
            [obj.x, obj.P] = obj.repropagate(x_corrected, P_corrected, idx_delayed);
        end
        
        function innovation = correctRadar(obj, z_radar)
            % Correction step with RADAR measurement (no delay)
            % z_radar: RADAR position measurement (3x1) - measures p_r directly
            %
            % Returns: innovation (3x1)
            
            % Use current state (RADAR has no delay)
            x_prior = obj.x;
            P_prior = obj.P;
            
            % Kalman gain (using RADAR measurement matrix)
            H = obj.H_radar;
            R = obj.R_radar;
            S = H * P_prior * H' + R;
            K = P_prior * H' / S;
            
            % Innovation
            z_pred = H * x_prior;
            innovation = z_radar - z_pred;
            
            % Correct the state
            obj.x = x_prior + K * innovation;
            % Joseph form for numerical stability
            obj.P = (eye(18) - K * H) * P_prior * (eye(18) - K * H)' + K * R * K';
            
            % Normalize quaternion
            % obj.x(obj.idx_q) = obj.x(obj.idx_q) / norm(obj.x(obj.idx_q));
        end
        
        function x_pred = predictState(obj, x, omega_meas, a_body_meas)
            % State prediction using IMU measurements
            
            % Extract states
            q = x(obj.idx_q);
            p_r = x(obj.idx_pr);
            v_r = x(obj.idx_vr);
            pbar = x(obj.idx_pbar);
            bgyr = x(obj.idx_bgyr);
            bacc = x(obj.idx_bacc);

            R_b2e = quat2rotm(q');
            p_zc = obj.e3' * obj.R_b2c * R_b2e' * (-p_r);
            p_zc = max(p_zc, 0.1);

            dt = obj.dt_imu;
            
            % Bias removal from IMU measurement
            omega  = omega_meas - bgyr;
            a_body = a_body_meas - bacc;

            % Quaternion prediction using M(delta_q) matrix
            wx = omega(1); wy = omega(2); wz = omega(3);
            M_dq = [1,        -wx*dt/2, -wy*dt/2, -wz*dt/2;
                    wx*dt/2,   1,        wz*dt/2, -wy*dt/2;
                    wy*dt/2,  -wz*dt/2,  1,        wx*dt/2;
                    wz*dt/2,   wy*dt/2, -wx*dt/2,  1];
            q_pred = M_dq * q;
            % q_pred = q_pred / norm(q_pred);
            
            % Velocity prediction
            a_world = R_b2e * a_body + obj.g * obj.e3;
            v_r_pred = v_r + a_world * dt;
            
            % Position prediction
            p_r_pred = p_r + 0.5 * (v_r + v_r_pred) * dt;
            
            % Image feature prediction
            pbar_x = pbar(1); pbar_y = pbar(2);
            
            % Linear velocity contribution
            % p_zc = 30; 
            % % pbar_x = pbar_x + 100;
            Lv = [-1/p_zc,     0,    pbar_x/p_zc;
                   0,     -1/p_zc,   pbar_y/p_zc];
            v_cam = obj.R_b2c * R_b2e' * v_r;
            
            % Angular velocity contribution
            Lw = [pbar_x*pbar_y,    -(1+pbar_x^2),   pbar_y;
                  (1+pbar_y^2),    -pbar_x*pbar_y,  -pbar_x];
            omega_cam = obj.R_b2c * omega;

            pbar_pred = pbar + (Lv * v_cam + Lw * omega_cam) * dt;
            
            % Assemble predicted state
            x_pred = x;
            x_pred(obj.idx_q) = q_pred;
            x_pred(obj.idx_pr) = p_r_pred;
            x_pred(obj.idx_vr) = v_r_pred;
            x_pred(obj.idx_pbar) = pbar_pred;
        end
        
        function updateHistory(obj, omega_meas, a_meas, t)
            % Update history buffers
            
            % Shift history
            obj.x_history(:, 1:end-1) = obj.x_history(:, 2:end);
            obj.P_history(:, :, 1:end-1) = obj.P_history(:, :, 2:end);
            obj.imu_history(:, 1:end-1) = obj.imu_history(:, 2:end);
            obj.time_history(1:end-1) = obj.time_history(2:end);
            
            % Add current state to history
            obj.x_history(:, end) = obj.x;
            obj.P_history(:, :, end) = obj.P;
            obj.imu_history(:, end) = [omega_meas; a_meas];
            obj.time_history(end) = t;
        end
        
        function [x_reprop, P_reprop] = repropagate(obj, x_corrected, P_corrected, idx_start)
            % Re-propagate from corrected delayed state to current time
            
            x_reprop = x_corrected;
            P_reprop = P_corrected;
            
            for j = idx_start+1:obj.history_length
                % Get IMU data from history
                omega_meas = obj.imu_history(1:3, j);
                a_meas = obj.imu_history(4:6, j);
                
                % Propagate state
                x_reprop = obj.predictState(x_reprop, omega_meas, a_meas);
                
                % Compute Jacobians and propagate covariance
                [F, G] = compute_jacobians(x_reprop, omega_meas, a_meas, obj.dt_imu, obj.R_b2c, ...
                                           obj.idx_q,    obj.idx_pr,   obj.idx_vr, ...
                                           obj.idx_pbar, obj.idx_bgyr, obj.idx_bacc);
                P_reprop = F * P_reprop * F' + G * obj.Q * G';
                
                % Normalize quaternion
                % x_reprop(obj.idx_q) = x_reprop(obj.idx_q) / norm(x_reprop(obj.idx_q));
            end
        end
        
        % Getter methods for state components
        function q = getQuaternion(obj)
            q = obj.x(obj.idx_q);
        end
        
        function p_r = getRelativePosition(obj)
            p_r = obj.x(obj.idx_pr);
        end
        
        function v_r = getRelativeVelocity(obj)
            v_r = obj.x(obj.idx_vr);
        end
        
        function pbar = getImageFeatures(obj)
            pbar = obj.x(obj.idx_pbar);
        end
        
        function b_gyr = getGyroBias(obj)
            b_gyr = obj.x(obj.idx_bgyr);
        end
        
        function b_acc = getAccelBias(obj)
            b_acc = obj.x(obj.idx_bacc);
        end
    end
    
    methods (Static)
        function x_init = createInitialState(q_true, p_r_true, v_r_true, pbar_true, ...
                                             b_gyr_true, b_acc_true, errors)
            % CREATEINITIALSTATE Create initial state estimate with errors
            %
            % Inputs:
            %   q_true     - True quaternion (4x1)
            %   p_r_true   - True relative position (3x1)
            %   v_r_true   - True relative velocity (3x1)
            %   pbar_true  - True image features (2x1)
            %   b_gyr_true - True gyro bias (3x1)
            %   b_acc_true - True accel bias (3x1)
            %   errors     - Struct with error fields (optional):
            %                  .euler_deg  - Euler angle errors [yaw; pitch; roll] in degrees
            %                  .position   - Position errors (3x1) in meters
            %                  .velocity   - Velocity errors (3x1) in m/s
            %                  .pbar       - Image feature errors (2x1)
            %                  .b_gyr      - Gyro bias errors (3x1)
            %                  .b_acc      - Accel bias errors (3x1)
            %
            % Output:
            %   x_init - Initial state estimate (18x1)
            
            % Default errors to zero
            if nargin < 7
                errors = struct();
            end
            
            % Quaternion initialization from Euler angle errors (more intuitive)
            if isfield(errors, 'euler_deg') && ~isempty(errors.euler_deg)
                % Convert euler errors to radians
                euler_err_rad = errors.euler_deg * pi / 180;  % [yaw; pitch; roll]
                
                % Get true Euler angles
                eul_true = quat2eul(q_true', 'ZYX')';  % [yaw; pitch; roll]
                
                % Add errors to Euler angles
                eul_est = eul_true + euler_err_rad;
                
                % Convert back to quaternion
                q_est = eul2quat(eul_est', 'ZYX')';
            else
                q_est = q_true;
            end
            % q_est = q_est / norm(q_est);
            
            % Position initialization
            if isfield(errors, 'position') && ~isempty(errors.position)
                p_r_est = p_r_true + errors.position;
            else
                p_r_est = p_r_true;
            end
            
            % Velocity initialization
            if isfield(errors, 'velocity') && ~isempty(errors.velocity)
                v_r_est = v_r_true + errors.velocity;
            else
                v_r_est = v_r_true;
            end
            
            % Image feature initialization
            if isfield(errors, 'pbar') && ~isempty(errors.pbar)
                pbar_est = pbar_true + errors.pbar;
            else
                pbar_est = pbar_true;
            end
            
            % Gyro bias initialization
            if isfield(errors, 'b_gyr') && ~isempty(errors.b_gyr)
                b_gyr_est = b_gyr_true + errors.b_gyr;
            else
                b_gyr_est = b_gyr_true;
            end
            
            % Accel bias initialization
            if isfield(errors, 'b_acc') && ~isempty(errors.b_acc)
                b_acc_est = b_acc_true + errors.b_acc;
            else
                b_acc_est = b_acc_true;
            end
            
            % Assemble initial state
            x_init = [q_est; p_r_est; v_r_est; pbar_est; b_gyr_est; b_acc_est];
        end
        
        function P_init = createInitialCovariance(sigma)
            % CREATEINITIALCOVARIANCE Create initial covariance matrix
            %
            % Input:
            %   sigma - Struct with standard deviation fields (optional):
            %             .quaternion - Quaternion std (scalar or 4x1)
            %             .position   - Position std (scalar or 3x1) [m]
            %             .velocity   - Velocity std (scalar or 3x1) [m/s]
            %             .pbar       - Image feature std (scalar or 2x1)
            %             .b_gyr      - Gyro bias std (scalar or 3x1) [rad/s]
            %             .b_acc      - Accel bias std (scalar or 3x1) [m/s^2]
            %
            % Output:
            %   P_init - Initial covariance matrix (18x18)
            
            % Default values
            if nargin < 1 || isempty(sigma)
                sigma = struct();
            end
            
            % Set default standard deviations
            sig_q = 0.1;      % Quaternion
            sig_p = 2;        % Position [m]
            sig_v = 0.3;      % Velocity [m/s]
            sig_pbar = 0.1;   % Image feature
            sig_bgyr = 0.01;  % Gyro bias [rad/s]
            sig_bacc = 0.1;   % Accel bias [m/s^2]
            
            if isfield(sigma, 'quaternion'), sig_q = sigma.quaternion; end
            if isfield(sigma, 'position'), sig_p = sigma.position; end
            if isfield(sigma, 'velocity'), sig_v = sigma.velocity; end
            if isfield(sigma, 'pbar'), sig_pbar = sigma.pbar; end
            if isfield(sigma, 'b_gyr'), sig_bgyr = sigma.b_gyr; end
            if isfield(sigma, 'b_acc'), sig_bacc = sigma.b_acc; end
            
            % Build covariance matrix
            P_init = diag([sig_q^2 * ones(1, 4), ...
                          sig_p^2 * ones(1, 3), ...
                          sig_v^2 * ones(1, 3), ...
                          sig_pbar^2 * ones(1, 2), ...
                          sig_bgyr^2 * ones(1, 3), ...
                          sig_bacc^2 * ones(1, 3)]);
        end
    end
end
