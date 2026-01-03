classdef ErrorStateKalmanFilter < handle
    % ErrorStateKalmanFilter - Error-State Kalman Filter for Visual Guidance
    %
    % ESKF maintains separation between:
    %   - Nominal state  x (18): [ q(4),  pr(3),  vr(3),  pbar(2),  bgyr(3),  bacc(3)]
    %   - Error   state δx (17): [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]
    %
    % The covariance P is 17x17 for the error state.
    
    properties
        % Nominal state indices (18-state)
        idx_q     = 1:4;      % Quaternion
        idx_pr    = 5:7;      % Relative position
        idx_vr    = 8:10;     % Relative velocity
        idx_pbar  = 11:12;    % Normalized image coordinates
        idx_bgyr  = 13:15;    % Gyroscope bias
        idx_bacc  = 16:18;    % Accelerometer bias
        
        % Error state indices (17-state)
        idx_dtheta = 1:3;     % Attitude error (axis-angle)
        idx_dpr    = 4:6;     % Position error
        idx_dvr    = 7:9;     % Velocity error
        idx_dpbar  = 10:11;   % Image feature error
        idx_dbgyr  = 12:14;   % Gyro bias error
        idx_dbacc  = 15:17;   % Accel bias error
        
        % Nominal state (18x1)
        x
        
        % Error state covariance (17x17)
        P
        
        % Physical constants
        g = 9.81;
        e3 = [0; 0; 1];
        
        % Camera parameters
        R_b2c
        
        % Noise parameters
        Qc                    % Continuous-time process noise (12x12)
        Qd                    % Discrete-time process noise (12x12)
        R_img                 % Image measurement noise (2x2)
        R_radar               % RADAR measurement noise (3x3)
        
        % Timing
        dt_imu
        
        % History buffers for delay handling
        history_length
        x_history             % Nominal state history (18xN)
        P_history             % Error covariance history (17x17xN)
        imu_history           % IMU measurement history (6xN)
        time_history
        
        % Measurement matrices for error state
        H_img_err             % Image measurement Jacobian wrt error state (2x17)
        H_radar_err           % RADAR measurement Jacobian wrt error state (3x17)
    end
    
    methods
        function obj = ErrorStateKalmanFilter(params)
            % Constructor
            
            obj.dt_imu = params.dt_imu;
            obj.R_b2c  =  params.R_b2c;
            
            % Process noise (continuous-time)
            % n = [wn(3), an(3), ww(3), aw(3)]
            obj.Qc = params.Qc;
            % obj.Qd = params.Qc * params.dt_imu;  % Simple discretization  % NOTE: correct that w.r.t. noise density and random walk
            obj.Qd = params.Qd;
            
            obj.R_img   = params.R_img;
            obj.R_radar = params.R_radar;
            
            % Initialize nominal state
            obj.x = params.x_init;
            
            % Initialize error covariance (17x17)
            obj.P = params.P_init;
            
            % Setup history buffers
            obj.history_length = max(params.delay_steps + 10, 15);
            obj.x_history = zeros(18, obj.history_length);
            obj.P_history = zeros(17, 17, obj.history_length);
            obj.imu_history = zeros(6, obj.history_length);
            obj.time_history = zeros(1, obj.history_length);
            
            for i = 1:obj.history_length
                obj.x_history(:, i) = obj.x;
                obj.P_history(:, :, i) = obj.P;
            end
            
            % Image measurement matrix: z = pbar, so H maps δpbar
            obj.H_img_err = zeros(2, 17);
            obj.H_img_err(1:2, obj.idx_dpbar) = eye(2);
            
            % RADAR measurement matrix: z = p_r, so H maps δpr
            obj.H_radar_err = zeros(3, 17);
            obj.H_radar_err(1:3, obj.idx_dpr) = eye(3);
        end
        
        function predict(obj, omega_meas, a_meas, t)
            % ESKF Prediction Step
            % 1. Propagate nominal state
            % 2. Propagate error covariance (error state stays at zero)
            
            % Propagate nominal state
            x_nom_new = obj.predictNominalState(obj.x, omega_meas, a_meas);
            
            % Compute ESKF Jacobians
            [~, Gc, Fd, Gd] = compute_eskf_jacobians(obj.x, omega_meas, a_meas, ...
                                                     obj.dt_imu, obj.R_b2c);
            
            % Propagate error covariance  % NOTE: CHECK Gd
            % P_k+1 = Fd * P_k * Fd' + Gd * Qc * Gd'
            % P_new = Fd * obj.P * Fd' + Gd * obj.Qc * Gd';
            P_new = Fd * obj.P * Fd' + Gc * obj.Qd * Gc';

            
            % Update
            obj.x = x_nom_new;
            obj.P = P_new;
            
            % Update history
            obj.updateHistory(omega_meas, a_meas, t);
        end
        
        function innovation = correctImage(obj, z_meas, delay_steps)
            % Correction with delayed IMAGE measurement
            % z_meas: measured pbar (2x1)
            % delay_steps: delay in IMU cycles
            
            innovation = [];
            
            idx_delayed = obj.history_length - delay_steps;
            if idx_delayed < 1
                return;
            end
            
            % Get prior estimates from history
            x_prior = obj.x_history(:, idx_delayed);
            P_prior = obj.P_history(:, :, idx_delayed);
            
            % Predicted measurement
            z_pred = x_prior(obj.idx_pbar);
            
            % Innovation
            innovation = z_meas - z_pred;
            
            % Kalman gain (error state space)
            H = obj.H_img_err;
            R = obj.R_img;
            S = H * P_prior * H' + R;
            K = P_prior * H' / S;
            
            % Compute error state correction
            delta_x = K * innovation;
            
            % Apply correction to nominal state
            x_corrected = obj.injectErrorState(x_prior, delta_x);
            
            % Update covariance (Joseph form)
            I_KH = eye(17) - K * H;
            P_corrected = I_KH * P_prior * I_KH' + K * R * K';
            
            % ESKF Reset: After injecting error into nominal, reset error covariance
            % P <- G * P * G'  where G accounts for attitude reset
            P_corrected = obj.resetCovariance(P_corrected, delta_x);
            
            % Re-propagate to current time
            [obj.x, obj.P] = obj.repropagate(x_corrected, P_corrected, idx_delayed);
        end
        
        function innovation = correctRadar(obj, z_radar)
            % Correction with RADAR measurement (no delay)
            % z_radar: measured p_r (3x1)
            
            x_prior = obj.x;
            P_prior = obj.P;
            
            % Predicted measurement
            z_pred = x_prior(obj.idx_pr);
            
            % Innovation
            innovation = z_radar - z_pred;
            
            % Kalman gain
            H = obj.H_radar_err;
            R = obj.R_radar;
            S = H * P_prior * H' + R;
            K = P_prior * H' / S;
            
            % Compute error state correction
            delta_x = K * innovation;
            
            % Apply correction to nominal state
            obj.x = obj.injectErrorState(x_prior, delta_x);
            
            % Update covariance (Joseph form)
            I_KH = eye(17) - K * H;
            P_updated = I_KH * P_prior * I_KH' + K * R * K';
            
            % ESKF Reset: After injecting error into nominal, reset error covariance
            % P <- G * P * G'  where G accounts for attitude reset
            obj.P = obj.resetCovariance(P_updated, delta_x);
        end
        
        function x_new = predictNominalState(obj, x, omega_meas, a_meas)
            % Propagate nominal state using IMU measurements
            
            q = x(obj.idx_q);
            p_r = x(obj.idx_pr);
            v_r = x(obj.idx_vr);
            pbar = x(obj.idx_pbar);
            b_gyr = x(obj.idx_bgyr);
            b_acc = x(obj.idx_bacc);
            
            dt = obj.dt_imu;
            
            % Corrected IMU
            omega  = omega_meas - b_gyr;
            a_body = a_meas - b_acc;
            
            % Rotation matrix
            R_b2e = quat2rotm(q');
            
            % Quaternion update using exponential map
            omega_dt = omega * dt;
            dq = obj.expQuaternion(omega_dt);
            q_new = quatmultiply(q', dq')';
            q_new = q_new / norm(q_new);
            
            % Velocity update
            a_world = R_b2e * a_body + obj.g * obj.e3;
            v_r_new = v_r + a_world * dt;
            
            % Position update (trapezoidal)
            p_r_new = p_r + 0.5 * (v_r + v_r_new) * dt;
            
            % Image feature update
            R_e2b = R_b2e';
            p_c = obj.R_b2c * R_e2b * (-p_r);
            p_c_z = max(p_c(3), 0.1);
            
            v_c = obj.R_b2c * R_e2b * v_r;
            w_c = obj.R_b2c * omega;
            
            pbar_x = pbar(1); pbar_y = pbar(2);
            
            Lv = [-1/p_c_z, 0,       pbar_x/p_c_z;
                   0,      -1/p_c_z, pbar_y/p_c_z];
            Lw = [pbar_x*pbar_y, -(1+pbar_x^2),   pbar_y;
                  (1+pbar_y^2),  -pbar_x*pbar_y, -pbar_x];
            
            pbar_dot = Lv * v_c + Lw * w_c;
            pbar_new = pbar + pbar_dot * dt;
            
            % Biases don't change in nominal propagation
            b_gyr_new = b_gyr;
            b_acc_new = b_acc;
            
            % Assemble new state
            x_new = [q_new; p_r_new; v_r_new; pbar_new; b_gyr_new; b_acc_new];
        end
        
        function x_corrected = injectErrorState(obj, x_nominal, delta_x)
            % Inject error state into nominal state
            % This is the key ESKF operation: x_true ≈ x_nominal ⊕ δx
            
            x_corrected = x_nominal;
            
            % Attitude: q_corrected = q_nominal ⊗ δq(δθ)
            delta_theta = delta_x(obj.idx_dtheta);
            dq = obj.expQuaternion(delta_theta);
            q_nom = x_nominal(obj.idx_q);
            q_corrected = quatmultiply(q_nom', dq')';
            q_corrected = q_corrected / norm(q_corrected);
            x_corrected(obj.idx_q) = q_corrected;
            
            % Position: additive
            x_corrected(obj.idx_pr) = x_nominal(obj.idx_pr) + delta_x(obj.idx_dpr);
            
            % Velocity: additive
            x_corrected(obj.idx_vr) = x_nominal(obj.idx_vr) + delta_x(obj.idx_dvr);
            
            % Image features: additive
            x_corrected(obj.idx_pbar) = x_nominal(obj.idx_pbar) + delta_x(obj.idx_dpbar);
            
            % Biases: additive
            x_corrected(obj.idx_bgyr) = x_nominal(obj.idx_bgyr) + delta_x(obj.idx_dbgyr);
            x_corrected(obj.idx_bacc) = x_nominal(obj.idx_bacc) + delta_x(obj.idx_dbacc);
        end
        
        function [x_final, P_final] = repropagate(obj, x_start, P_start, idx_start)
            % Re-propagate from corrected delayed state to current time
            
            x_reprop = x_start;
            P_reprop = P_start;
            
            for j = idx_start:obj.history_length-1
                omega_meas = obj.imu_history(1:3, j);
                a_meas = obj.imu_history(4:6, j);
                
                % Propagate nominal state
                x_reprop = obj.predictNominalState(x_reprop, omega_meas, a_meas);
                
                % Propagate covariance
                [~, Gc, Fd, Gd] = compute_eskf_jacobians(x_reprop, omega_meas, a_meas, ...
                                                         obj.dt_imu, obj.R_b2c);
                % P_reprop = Fd * P_reprop * Fd' + Gd * obj.Qc * Gd';
                P_reprop = Fd * P_reprop * Fd' + Gc * obj.Qd * Gc';

            end
            
            x_final = x_reprop;
            P_final = P_reprop;
        end
        
        function updateHistory(obj, omega_meas, a_meas, t)
            % Shift history left (discard oldest at position 1, add newest at end)
            obj.x_history(:, 1:end-1)      = obj.x_history(:, 2:end);
            obj.P_history(:, :, 1:end-1)   = obj.P_history(:, :, 2:end);
            obj.imu_history(:, 1:end-1)    = obj.imu_history(:, 2:end);
            obj.time_history(1:end-1)      = obj.time_history(2:end);
            
            % Add newest at end
            obj.x_history(:, end)      = obj.x;
            obj.P_history(:, :, end)   = obj.P;
            obj.imu_history(:, end)    = [omega_meas; a_meas];
            obj.time_history(end)      = t;
        end
        
        function dq = expQuaternion(~, delta_theta)
            % Convert small rotation vector to quaternion using exponential map
            % dq = [cos(|δθ|/2); sin(|δθ|/2) * δθ/|δθ|]
            
            theta = norm(delta_theta);
            
            if theta < 1e-10
                dq = [1; 0; 0; 0];
            else
                u = delta_theta / theta;
                dq = [cos(theta/2); sin(theta/2) * u];
            end
        end
        
        function P_reset = resetCovariance(obj, P, delta_x)
            % ESKF Covariance Reset after error state injection
            % P_reset = G * P * G'
            %
            % The G matrix accounts for the attitude parametrization:
            % For our state order: δx = [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]
            %
            % G = blkdiag(I₃ - ½[δθ]×, I₃, I₃, I₂, I₃, I₃)
            %
            % For small δθ (after injection), G ≈ I₁₇
            
            delta_theta = delta_x(obj.idx_dtheta);
            
            % Build G matrix (17x17)
            G = eye(17);
            
            % Attitude block: I - ½[δθ]×
            G(1:3, 1:3) = eye(3) - 0.5 * skew(delta_theta);
            
            % Apply reset
            P_reset = G * P * G';
        end
    end
    
    methods (Static)
        function x_init = createInitialState(q_true, p_r_true, v_r_true, pbar_true, ...
                                              b_gyr_true, b_acc_true, errors)
            % Create initial nominal state with errors
            % Same as DKF for compatibility
            
            % Attitude error
            if isfield(errors, 'euler_deg') && ~isempty(errors.euler_deg)
                euler_error_rad = errors.euler_deg * pi/180;
                dq = eul2quat(euler_error_rad', 'ZYX')';
                q_est = quatmultiply(q_true', dq')';
            else
                q_est = q_true;
            end
            q_est = q_est / norm(q_est);
            
            % Position
            if isfield(errors, 'position') && ~isempty(errors.position)
                p_r_est = p_r_true + errors.position;
            else
                p_r_est = p_r_true;
            end
            
            % Velocity
            if isfield(errors, 'velocity') && ~isempty(errors.velocity)
                v_r_est = v_r_true + errors.velocity;
            else
                v_r_est = v_r_true;
            end
            
            % Image features
            if isfield(errors, 'pbar') && ~isempty(errors.pbar)
                pbar_est = pbar_true + errors.pbar;
            else
                pbar_est = pbar_true;
            end
            
            % Biases (start at zero if not specified)
            if isfield(errors, 'b_gyr') && ~isempty(errors.b_gyr)
                b_gyr_est = errors.b_gyr;
            else
                b_gyr_est = zeros(3, 1);
            end
            
            if isfield(errors, 'b_acc') && ~isempty(errors.b_acc)
                b_acc_est = errors.b_acc;
            else
                b_acc_est = zeros(3, 1);
            end
            
            x_init = [q_est; p_r_est; v_r_est; pbar_est; b_gyr_est; b_acc_est];
        end
        
        function P_init = createInitialCovariance(sigmas)
            % Create initial 17x17 error covariance
            
            P_init = diag([
                sigmas.attitude^2 * ones(1, 3), ...    % δθ
                sigmas.position^2 * ones(1, 3), ...    % δpr
                sigmas.velocity^2 * ones(1, 3), ...    % δvr
                sigmas.pbar^2 * ones(1, 2), ...        % δpbar
                sigmas.b_gyr^2 * ones(1, 3), ...       % δbgyr
                sigmas.b_acc^2 * ones(1, 3)            % δbacc
            ]);
        end
    end
end

%% Helper function (outside class)
function S = skew(v)
    % Skew-symmetric matrix from 3-vector
    % S*x = v × x
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end
