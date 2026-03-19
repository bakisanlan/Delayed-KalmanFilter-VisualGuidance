classdef ErrorStateKalmanFilter_reduced < handle
    % ErrorStateKalmanFilter - Error-State Kalman Filter for Visual Guidance
    %
    % ESKF maintains separation between:
    %   - Nominal state  x (8): [ pt(3),  vt(3),  pbar(2)]
    %   - Error   state δx (8): [δpt(3), δvt(3), δpbar(2)]
    %
    % The covariance P is 8x8 for the error state.
    
    properties
        % Nominal state indices (8-state)
        idx_pt    = 1:3;      % Target position
        idx_vt    = 4:6;      % Target velocity
        idx_pbar  = 7:8;      % Normalized image coordinates

        
        % Error state indices (8-state)
        idx_dpt    = 1:3;     % Target position error
        idx_dvt    = 4:6;     % Target velocity error
        idx_dpbar  = 7:8;     % Image feature error

        
        % Nominal state (8x1)
        x
        
        % Error state covariance (8x8)
        P
        
        % Physical constants
        g = 9.81;
        e3 = [0; 0; 1];
        
        % Camera parameters
        R_b2c
        
        % Noise parameters
        Qc                    % Continuous-time process noise (3x3)
        Qd                    % Discrete-time process noise (3x3)
        R_img                 % Image measurement noise (2x2)
        R_radar               % RADAR measurement noise (6x6) - [pt; vt]
        
        % Timing
        dt_imu                % IMU sampling interval
        dt_eskf               % ESKF update interval (can differ from dt_imu)
        
        % History buffers for delay handling
        history_length
        x_history             % Nominal state history (8xN)
        P_history             % Error covariance history (8x8xN)
        imu_history           % IMU measurement history (6xN)
        R_b2e_history         % Interceptor attitude history (3x3xN)
        p_i_history           % Interceptor position history (3xN)
        v_i_history           % Interceptor velocity history (3xN)
        time_history
        
        % Measurement matrices for error state
        H_img_err             % Image measurement Jacobian wrt error state (2x8)
        H_radar_err           % RADAR measurement Jacobian wrt error state (6x8)
        
    end
    
    methods
        function obj = ErrorStateKalmanFilter_reduced(params)
            % Constructor
            
            obj.dt_imu = params.dt_imu;
            obj.dt_eskf = params.dt_eskf;
            obj.R_b2c  =  params.R_b2c;
            
            % Process noise (continuous-time)
            % n = [vn(3)]
            if isfield(params, 'Qc')
                obj.Qc = params.Qc;
            else
                obj.Qc = eye(3);
            end

            if isfield(params, 'Qd')
                obj.Qd = params.Qd;
            else
                obj.Qd = obj.Qc * obj.dt_eskf;
            end
            
            obj.R_img   = params.R_img;
            obj.R_radar = params.R_radar;
            
            % Initialize nominal state
            obj.x = params.x_init;
            
            % Initialize error covariance (8x8)
            obj.P = params.P_init;
            
            % Setup history buffers
            obj.history_length = max(params.delay_steps + 10, 15);
            obj.x_history = zeros(8, obj.history_length);
            obj.P_history = zeros(8, 8, obj.history_length);
            obj.imu_history = zeros(6, obj.history_length);
            obj.R_b2e_history = repmat(eye(3), 1, 1, obj.history_length);
            obj.p_i_history = zeros(3, obj.history_length);
            obj.v_i_history = zeros(3, obj.history_length);
            obj.time_history = zeros(1, obj.history_length);
            
            for i = 1:obj.history_length
                obj.x_history(:, i) = obj.x;
                obj.P_history(:, :, i) = obj.P;
            end
            
            % Image measurement matrix: z = pbar, so H maps δpbar
            obj.H_img_err = zeros(2, 8);
            obj.H_img_err(1:2, obj.idx_dpbar) = eye(2);
            
            % RADAR measurement matrix: z = [p_t; v_t], so H maps [δpt; δvt]
            obj.H_radar_err = zeros(6, 8);
            obj.H_radar_err(1:3, obj.idx_dpt) = eye(3);  % Position
            obj.H_radar_err(4:6, obj.idx_dvt) = eye(3);  % Velocity
        end
        
        function predict(obj, omega_meas, a_meas, t, R_b2e, p_i, v_i)
            % ESKF Prediction Step
            % 1. Propagate nominal state
            % 2. Propagate error covariance (error state stays at zero)
            
            % Propagate nominal state
            x_nom_new = obj.predictNominalState(obj.x, omega_meas, R_b2e, p_i, v_i, obj.dt_eskf);
            
            % Compute ESKF Jacobians using dt_eskf
            [~, Gc, Fd, ~] = compute_eskf_jacobians_reduced(obj.x, omega_meas, ...
                                                            obj.dt_eskf, obj.R_b2c, ...
                                                            R_b2e, p_i, v_i);

            % % Scale process noise by dt_eskf ratio or Gd is already
            % calculated according to dt_eskf
            % dt_ratio = obj.dt_eskf / obj.dt_imu;
            % Qd_scaled = obj.Qd * dt_ratio;
            
            % Propagate error covariance
            P_new = Fd * obj.P * Fd' + Gc * obj.Qd * Gc';

            % Update
            obj.x = x_nom_new;
            obj.P = P_new;
            
            % Update history
            obj.updateHistory(omega_meas, a_meas, t, R_b2e, p_i, v_i);
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

            % === Mahalanobis Distance Chi-Square Gating ===
            % Squared Mahalanobis distance: d² = y' * S^(-1) * y
            % For 2D measurement, d² follows chi-square distribution with 2 DoF
            d_mahal_sq = innovation' * (S \ innovation);
            
            % Chi-square threshold for 99.99% confidence with 2 DoF
            % chi2inv(0.9999, 2) ≈ 18.42
            chi2_threshold = 18.42;
            
            if d_mahal_sq > chi2_threshold
                fprintf('Cam False detection rejected (Mahalanobis d²=%.2f > %.2f)\n', ...
                        d_mahal_sq, chi2_threshold);
                return
            end

            K = P_prior * H' / S;
            
            % Compute error state correction
            delta_x = K * innovation;
            
            % Apply correction to nominal state
            x_corrected = obj.injectErrorState(x_prior, delta_x);
            
            % Update covariance (Joseph form)
            I_KH = eye(8) - K * H;
            P_corrected = I_KH * P_prior * I_KH' + K * R * K';
            
            % ESKF Reset: After injecting error into nominal, reset error covariance
            % P <- G * P * G'  where G accounts for attitude reset
            P_corrected = obj.resetCovariance(P_corrected);
            
            % Re-propagate to current time
            [obj.x, obj.P] = obj.repropagate(x_corrected, P_corrected, idx_delayed);
        end
        
        function innovation = correctRadar(obj, z_radar)
            % Correction with RADAR measurement (no delay)
            % z_radar: measured [p_t; v_t] (6x1)
            
            x_prior = obj.x;
            P_prior = obj.P;
            
            % Predicted measurement [p_t; v_t]
            z_pred = [x_prior(obj.idx_pt); x_prior(obj.idx_vt)];
            
            % Innovation
            innovation = z_radar - z_pred;
            
            % Kalman gain
            H = obj.H_radar_err;
            R = obj.R_radar;
            S = H * P_prior * H' + R;
            
            % === Mahalanobis Distance Chi-Square Gating ===
            % Squared Mahalanobis distance: d² = y' * S^(-1) * y
            % For 6D measurement, d² follows chi-square distribution with 6 DoF
            d_mahal_sq = innovation' * (S \ innovation);
            
            % Chi-square threshold for 99.99% confidence with 6 DoF
            % chi2inv(0.9999, 6) ≈ 27.86
            chi2_threshold = 27.86;
            
            % if d_mahal_sq > chi2_threshold
            %     fprintf('[ESKF] RADAR false detection rejected (Mahalanobis d²=%.2f > %.2f)\n', ...
            %             d_mahal_sq, chi2_threshold);
            %     return
            % end
            
            K = P_prior * H' / S;
            
            % Compute error state correction
            delta_x = K * innovation;
            
            % Apply correction to nominal state
            obj.x = obj.injectErrorState(x_prior, delta_x);
            
            % Update covariance (Joseph form for 6x6 R)
            I_KH = eye(8) - K * H;
            P_updated = I_KH * P_prior * I_KH' + K * R * K';
            
            % ESKF Reset: After injecting error into nominal, reset error covariance
            % P <- G * P * G'  where G accounts for attitude reset
            obj.P = obj.resetCovariance(P_updated);
        end
        
        function x_new = predictNominalState(obj, x, omega_meas, R_b2e, p_i, v_i, dt)
            % Propagate nominal state using IMU measurements
            % dt: optional time step (defaults to dt_eskf)
            
            if nargin < 8
                dt = obj.dt_eskf;
            end
            
            p_t = x(obj.idx_pt);
            v_t = x(obj.idx_vt);
            pbar = x(obj.idx_pbar);

            % Corrected IMU
            omega  = omega_meas;
            % a_body = a_meas;
            
            % Position update (trapezoidal)
            p_t_new = p_t + v_t * dt;

            % Velocity update (no info)
            v_t_new = v_t;
            
            % Image feature update
            R_e2b = R_b2e';
            p_c = obj.R_b2c * R_e2b * (-(p_i - p_t));
            p_c_z = max(p_c(3), 0.1);
            
            v_c = obj.R_b2c * R_e2b * (v_i - v_t);
            w_c = obj.R_b2c * omega;
            
            pbar_x = pbar(1); pbar_y = pbar(2);
            
            Lv = [-1/p_c_z, 0,       pbar_x/p_c_z;
                   0,      -1/p_c_z, pbar_y/p_c_z];
            Lw = [pbar_x*pbar_y, -(1+pbar_x^2),   pbar_y;
                  (1+pbar_y^2),  -pbar_x*pbar_y, -pbar_x];
            
            pbar_dot = Lv * v_c + Lw * w_c;
            pbar_new = pbar + pbar_dot * dt;
            
            % Assemble new state
            x_new = [p_t_new; v_t_new; pbar_new];
        end
        
        
        function x_corrected = injectErrorState(obj, x_nominal, delta_x)
            % Inject error state into nominal state
            % This is the key ESKF operation: x_true ≈ x_nominal ⊕ δx
            
            x_corrected = x_nominal;
            
            % Position: additive
            x_corrected(obj.idx_pt) = x_nominal(obj.idx_pt) + delta_x(obj.idx_dpt);
            
            % Velocity: additive
            x_corrected(obj.idx_vt) = x_nominal(obj.idx_vt) + delta_x(obj.idx_dvt);
            
            % Image features: additive
            x_corrected(obj.idx_pbar) = x_nominal(obj.idx_pbar) + delta_x(obj.idx_dpbar); 
        end
        
        function [x_final, P_final] = repropagate(obj, x_start, P_start, idx_start)
            % Re-propagate from corrected delayed state to current time
            
            x_reprop = x_start;
            P_reprop = P_start;
            
            for j = idx_start:obj.history_length-1
                omega_meas = obj.imu_history(1:3, j);
                R_b2e_j = obj.R_b2e_history(:, :, j);
                p_i_j = obj.p_i_history(:, j);
                v_i_j = obj.v_i_history(:, j);
                
                % Propagate nominal state using dt_eskf for history playback
                x_reprop = obj.predictNominalState(x_reprop, omega_meas, R_b2e_j, ...
                                                   p_i_j, v_i_j, obj.dt_eskf);
                
                % Propagate covariance
                [~, Gc, Fd, ~] = compute_eskf_jacobians_reduced(x_reprop, omega_meas, ...
                                                                obj.dt_eskf, obj.R_b2c,...
                                                                R_b2e_j, p_i_j, v_i_j);
                % P_reprop = Fd * P_reprop * Fd' + Gd * obj.Qc * Gd';
                P_reprop = Fd * P_reprop * Fd' + Gc * obj.Qd * Gc';

            end
            
            x_final = x_reprop;
            P_final = P_reprop;
        end
        
        function updateHistory(obj, omega_meas, a_meas, t, R_b2e, p_i, v_i)
            % Shift history left (discard oldest at position 1, add newest at end)
            obj.x_history(:, 1:end-1)      = obj.x_history(:, 2:end);
            obj.P_history(:, :, 1:end-1)   = obj.P_history(:, :, 2:end);
            obj.imu_history(:, 1:end-1)    = obj.imu_history(:, 2:end);
            obj.R_b2e_history(:, :, 1:end-1) = obj.R_b2e_history(:, :, 2:end);
            obj.p_i_history(:, 1:end-1)    = obj.p_i_history(:, 2:end);
            obj.v_i_history(:, 1:end-1)    = obj.v_i_history(:, 2:end);
            obj.time_history(1:end-1)      = obj.time_history(2:end);
            
            % Add newest at end
            obj.x_history(:, end)      = obj.x;
            obj.P_history(:, :, end)   = obj.P;
            obj.imu_history(:, end)    = [omega_meas; a_meas];
            obj.R_b2e_history(:, :, end) = R_b2e;
            obj.p_i_history(:, end)    = p_i;
            obj.v_i_history(:, end)    = v_i;
            obj.time_history(end)      = t;
        end
        
        function P_reset = resetCovariance(obj, P)
            % ESKF Covariance Reset after error state injection
            % P_reset = G * P * G'
            %
            % For our state order: δx = [δpt(3), δvt(3), δpbar(2)]
            %
            % G = blkdiag(I₃, I₃, I₂)
            %
            
            % Build G matrix (8x8)
            G = eye(8);
            
            % Apply reset
            P_reset = G * P * G';
        end
    end
    
    methods (Static)
        function x_init = createInitialState(p_t_true, v_t_true, pbar_true, errors)
            % Create initial nominal state with errors
            % Same as DKF for compatibility
            
            % Position
            if isfield(errors, 'position') && ~isempty(errors.position)
                p_t_est = p_t_true + errors.position;
            else
                p_t_est = p_t_true;
            end
            
            % Velocity
            if isfield(errors, 'velocity') && ~isempty(errors.velocity)
                v_t_est = v_t_true + errors.velocity;
            else
                v_t_est = v_t_true;
            end
            
            % Image features
            if isfield(errors, 'pbar') && ~isempty(errors.pbar)
                pbar_est = pbar_true + errors.pbar;
            else
                pbar_est = pbar_true;
            end
            
            x_init = [p_t_est; v_t_est; pbar_est];
        end
        
        function P_init = createInitialCovariance(sigmas)
            % Create initial 8x8 error covariance
            
            P_init = diag([
                sigmas.position^2 * ones(1, 3), ...    % δpr
                sigmas.velocity^2 * ones(1, 3), ...    % δvr
                sigmas.pbar^2 * ones(1, 2)        % δpbar
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
