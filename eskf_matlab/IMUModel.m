classdef IMUModel < handle
    % IMUMODEL Realistic IMU sensor model with noise and bias random walk
    %
    % Notation follows the paper convention:
    %   σ_ãn [m/s²]     : Accelerometer measurement noise std
    %   σ_ω̃n [rad/s]    : Gyroscope measurement noise std
    %   σ_aw [m/s²√s]   : Accelerometer bias random walk
    %   σ_ωw [rad/s√s]  : Gyroscope bias random walk
    %
    % Discrete noise covariances (Qi block diagonal):
    %   V_i = σ²_an * Δt² * I  [m²/s²]     - velocity noise covariance
    %   Θ_i = σ²_ωn * Δt² * I  [rad²]      - attitude noise covariance
    %   A_i = σ²_aw * Δt * I   [m²/s⁴]     - accel bias RW covariance
    %   Ω_i = σ²_ωw * Δt * I   [rad²/s²]   - gyro bias RW covariance
    %
    % Measurement model:
    %   ω_meas = ω_true + ω_b + n_ω,  where n_ω ~ N(0, σ²_ωn * I)
    %   a_meas = a_true + a_b + n_a,  where n_a ~ N(0, σ²_an * I)
    %
    % Bias random walk:
    %   ω_b(k+1) = ω_b(k) + ω_i,  where ω_i ~ N(0, Ω_i)
    %   a_b(k+1) = a_b(k) + a_i,  where a_i ~ N(0, A_i)
    
    properties
        % Sample rate
        fs              % Sample frequency [Hz]
        dt              % Sample period Δt [s]
        
        % Noise parameters (paper notation)
        sigma_a_n       % σ_ãn: Accelerometer noise std [m/s²]
        sigma_omega_n   % σ_ω̃n: Gyroscope noise std [rad/s]
        sigma_a_w       % σ_aw: Accelerometer bias random walk [m/s²√s]
        sigma_omega_w   % σ_ωw: Gyroscope bias random walk [rad/s√s]
        
        % Discrete noise covariances (computed from parameters)
        V_i             % Velocity noise cov: σ²_an * Δt² * I₃ [m²/s²]
        Theta_i         % Attitude noise cov: σ²_ωn * Δt² * I₃ [rad²]
        A_i             % Accel bias RW cov: σ²_aw * Δt * I₃ [m²/s⁴]
        Omega_i         % Gyro bias RW cov: σ²_ωw * Δt * I₃ [rad²/s²]
        
        % Current bias states (evolving)
        omega_b         % Current gyroscope bias ω_b [rad/s]
        a_b             % Current accelerometer bias a_b [m/s²]
        
        % Initial bias values
        omega_b_init
        a_b_init
    end
    
    methods
        function obj = IMUModel(params)
            % Constructor
            %
            % Required params:
            %   .fs             - Sample frequency [Hz]
            %
            % Optional params (with defaults):
            %   .sigma_a_n      - Accel noise std σ_ãn [m/s²] (default: 0.1)
            %   .sigma_omega_n  - Gyro noise std σ_ω̃n [rad/s] (default: 0.01)
            %   .sigma_a_w      - Accel bias RW σ_aw [m/s²√s] (default: 1e-4)
            %   .sigma_omega_w  - Gyro bias RW σ_ωw [rad/s√s] (default: 1e-5)
            %   .omega_b_init   - Initial gyro bias [rad/s] (default: zeros)
            %   .a_b_init       - Initial accel bias [m/s²] (default: zeros)
            
            % Sample rate (required)
            obj.fs = params.fs;
            obj.dt = 1 / obj.fs;
            
            % === Noise parameters with defaults (typical MEMS) ===
            % σ_ãn [m/s²] - Accelerometer measurement noise std
            if isfield(params, 'sigma_a_n')
                obj.sigma_a_n = params.sigma_a_n;
            else
                obj.sigma_a_n = 0.1;  % m/s² (typical MEMS)
            end
            
            % σ_ω̃n [rad/s] - Gyroscope measurement noise std
            if isfield(params, 'sigma_omega_n')
                obj.sigma_omega_n = params.sigma_omega_n;
            else
                obj.sigma_omega_n = 0.01;  % rad/s (~0.57 deg/s, typical MEMS)
            end
            
            % σ_aw [m/s²√s] - Accelerometer bias random walk
            if isfield(params, 'sigma_a_w')
                obj.sigma_a_w = params.sigma_a_w;
            else
                obj.sigma_a_w = 1e-4;  % m/s²√s
            end
            
            % σ_ωw [rad/s√s] - Gyroscope bias random walk  
            if isfield(params, 'sigma_omega_w')
                obj.sigma_omega_w = params.sigma_omega_w;
            else
                obj.sigma_omega_w = 1e-5;  % rad/s√s
            end
            
            % === Initial biases ===
            if isfield(params, 'omega_b_init')
                obj.omega_b_init = params.omega_b_init(:);
            else
                obj.omega_b_init = zeros(3, 1);
            end
            
            if isfield(params, 'a_b_init')
                obj.a_b_init = params.a_b_init(:);
            else
                obj.a_b_init = zeros(3, 1);
            end
            
            % Initialize current biases
            obj.omega_b = obj.omega_b_init;
            obj.a_b = obj.a_b_init;
            
            % % === Compute discrete noise covariances ===
            obj.computeDiscreteCovariances(obj.dt);
            
            % Print summary
            obj.printSummary();
        end
        
        function computeDiscreteCovariances(obj,dt_eskf)
            % Compute discrete noise covariances from paper formulas:
            %   V_i = σ²_an * Δt² * I₃   [m²/s²]
            %   Θ_i = σ²_ωn * Δt² * I₃   [rad²]
            %   A_i = σ²_aw * Δt * I₃    [m²/s⁴]
            %   Ω_i = σ²_ωw * Δt * I₃    [rad²/s²]
            
            % V_i: Velocity noise covariance (eq. 262)
            obj.V_i = obj.sigma_a_n^2 * dt_eskf^2 * eye(3);
            
            % Θ_i: Attitude noise covariance (eq. 263)
            obj.Theta_i = obj.sigma_omega_n^2 * dt_eskf^2 * eye(3);
            
            % A_i: Accel bias random walk covariance (eq. 264)
            obj.A_i = obj.sigma_a_w^2 * dt_eskf * eye(3);
            
            % Ω_i: Gyro bias random walk covariance (eq. 265)
            obj.Omega_i = obj.sigma_omega_w^2 * dt_eskf * eye(3);
        end
        
        function [omega_meas, a_meas, omega_b_out, a_b_out] = measure(obj, omega_true, a_true)
            % Generate noisy IMU measurement with evolving biases
            %
            % Inputs:
            %   omega_true - True angular velocity [rad/s] (3x1)
            %   a_true     - True specific force [m/s²] (3x1)
            %
            % Outputs:
            %   omega_meas - Measured angular velocity [rad/s] (3x1)
            %   a_meas     - Measured specific force [m/s²] (3x1)
            %   omega_b_out - Current gyro bias [rad/s] (3x1)
            %   a_b_out    - Current accel bias [m/s²] (3x1)
            %
            % Model:
            %   ω_meas = ω_true + ω_b + n_ω
            %   a_meas = a_true + a_b + n_a
            
            % Update biases via random walk:
            %   ω_b(k+1) = ω_b(k) + ω_i, where ω_i ~ N(0, Ω_i)
            %   a_b(k+1) = a_b(k) + a_i, where a_i ~ N(0, A_i)
            omega_i = sqrt(obj.Omega_i(1,1)) * randn(3, 1);
            a_i = sqrt(obj.A_i(1,1)) * randn(3, 1);
            
            obj.omega_b = obj.omega_b + omega_i;
            obj.a_b = obj.a_b + a_i;
            
            % Generate measurement noise
            %   n_ω ~ N(0, σ²_ωn * I), n_a ~ N(0, σ²_an * I)
            n_omega = obj.sigma_omega_n * randn(3, 1);
            n_a = obj.sigma_a_n * randn(3, 1);
            
            % Measured values = true + bias + noise
            omega_meas = omega_true + obj.omega_b + n_omega;
            a_meas = a_true + obj.a_b + n_a;
            
            % Output current biases
            omega_b_out = obj.omega_b;
            a_b_out = obj.a_b;
        end
        
        function reset(obj)
            % Reset biases to initial values
            obj.omega_b = obj.omega_b_init;
            obj.a_b = obj.a_b_init;
        end
        
        function Qi = getDiscreteProcessNoise(obj,dt_eskf)
            % Get discrete-time process noise covariance Qi (12x12)
            %
            % Qi = blkdiag(V_i, Θ_i, A_i, Ω_i)
            %
            % This is the covariance of the noise vector:
            %   n = [v_i(3), θ_i(3), a_i(3), ω_i(3)]
            %
            % Order: velocity noise, attitude noise, accel bias RW, gyro
            % bias RW  % NOTE: CHANGE ORDER
            
            % Qi = blkdiag(obj.V_i, obj.Theta_i, obj.A_i, obj.Omega_i);
            if nargin>1
                obj.computeDiscreteCovariances(dt_eskf);
            end

            Qi = blkdiag(obj.Theta_i, obj.V_i, obj.Omega_i, obj.A_i);

        end
        
        function Qc = getESKFProcessNoise(obj)
            % Get process noise for ESKF matching Gc matrix structure
            %
            % ESKF noise vector order: n = [n_ω(3), n_a(3), w_ω(3), w_a(3)]
            %   n_ω : gyro measurement noise
            %   n_a : accel measurement noise
            %   w_ω : gyro bias random walk
            %   w_a : accel bias random walk
            %
            % Returns Qc such that when used with Gd = Gc*dt:
            %   P = Fd*P*Fd' + Gd*Qc*Gd'
            %
            % Qc = diag([σ²_ωn, σ²_an, σ²_ωw, σ²_aw])
            
            Qc = diag([obj.sigma_omega_n^2 * ones(1,3), ...  % Gyro noise σ²_ωn
                       obj.sigma_a_n^2 * ones(1,3), ...      % Accel noise σ²_an
                       obj.sigma_omega_w^2 * ones(1,3), ...  % Gyro bias RW σ²_ωw
                       obj.sigma_a_w^2 * ones(1,3)]);        % Accel bias RW σ²_aw
        end
        
        function printSummary(obj)
            % Print IMU model configuration
            fprintf('\n============ IMU Model Configuration ============\n');
            fprintf('Sample Rate: %.0f Hz (Δt = %.4f s)\n', obj.fs, obj.dt);
            fprintf('\n--- Noise Parameters (Paper Notation) ---\n');
            fprintf('σ_ãn (accel noise):       %.4e m/s² (%.3f mg)\n', ...
                    obj.sigma_a_n, obj.sigma_a_n / 9.81 * 1000);
            fprintf('σ_ω̃n (gyro noise):        %.4e rad/s (%.4f deg/s)\n', ...
                    obj.sigma_omega_n, obj.sigma_omega_n * 180/pi);
            fprintf('σ_aw (accel bias RW):     %.4e m/s²√s\n', obj.sigma_a_w);
            fprintf('σ_ωw (gyro bias RW):      %.4e rad/s√s\n', obj.sigma_omega_w);
            fprintf('\n--- Discrete Covariances (at %.0f Hz) ---\n', obj.fs);
            fprintf('V_i = σ²_an·Δt²·I:   diag = %.4e m²/s²\n', obj.V_i(1,1));
            fprintf('Θ_i = σ²_ωn·Δt²·I:   diag = %.4e rad²\n', obj.Theta_i(1,1));
            fprintf('A_i = σ²_aw·Δt·I:    diag = %.4e m²/s⁴\n', obj.A_i(1,1));
            fprintf('Ω_i = σ²_ωw·Δt·I:    diag = %.4e rad²/s²\n', obj.Omega_i(1,1));
            fprintf('\n--- Initial Biases ---\n');
            fprintf('ω_b(0): [%.4f, %.4f, %.4f] rad/s\n', obj.omega_b_init');
            fprintf('a_b(0): [%.4f, %.4f, %.4f] m/s²\n', obj.a_b_init');
            fprintf('=================================================\n\n');
        end
    end
    
    methods (Static)
        function params = typicalMEMS()
            % Return typical MEMS IMU parameters (MPU-6050 class)
            params = struct();
            params.sigma_a_n = 0.1;         % m/s² (~10 mg noise)
            params.sigma_omega_n = 0.01;    % rad/s (~0.57 deg/s)
            params.sigma_a_w = 1e-4;        % m/s²√s
            params.sigma_omega_w = 1e-5;    % rad/s√s
            params.omega_b_init = [0.005; -0.003; 0.002];  % rad/s
            params.a_b_init = [0.02; -0.01; 0.015];        % m/s²
        end
        
        function params = tacticalGrade()
            % Return tactical-grade IMU parameters (ADIS16490 class)
            params = struct();
            params.sigma_a_n = 0.005;       % m/s² (~0.5 mg)
            params.sigma_omega_n = 0.001;   % rad/s (~0.06 deg/s)
            params.sigma_a_w = 1e-5;        % m/s²√s
            params.sigma_omega_w = 1e-6;    % rad/s√s
            params.omega_b_init = [0.0001; -0.0001; 0.0001];
            params.a_b_init = [0.001; -0.001; 0.001];
        end
        
        function params = navigationGrade()
            % Return navigation-grade IMU parameters (fiber optic gyro class)
            params = struct();
            params.sigma_a_n = 0.001;       % m/s² (~0.1 mg)
            params.sigma_omega_n = 0.0001;  % rad/s (~0.006 deg/s)
            params.sigma_a_w = 1e-6;        % m/s²√s
            params.sigma_omega_w = 1e-7;    % rad/s√s
            params.omega_b_init = zeros(3, 1);
            params.a_b_init = zeros(3, 1);
        end
        
        function compareGrades()
            % Compare different IMU grades
            fprintf('\n========= IMU Grade Comparison =========\n');
            fprintf('%-18s %-12s %-12s %-12s\n', 'Parameter', 'MEMS', 'Tactical', 'Navigation');
            fprintf('%-18s %-12s %-12s %-12s\n', '---------', '----', '--------', '----------');
            
            mems = IMUModel.typicalMEMS();
            tact = IMUModel.tacticalGrade();
            nav = IMUModel.navigationGrade();
            
            fprintf('%-18s %-12.1e %-12.1e %-12.1e\n', 'σ_ãn [m/s²]', ...
                    mems.sigma_a_n, tact.sigma_a_n, nav.sigma_a_n);
            fprintf('%-18s %-12.1e %-12.1e %-12.1e\n', 'σ_ω̃n [rad/s]', ...
                    mems.sigma_omega_n, tact.sigma_omega_n, nav.sigma_omega_n);
            fprintf('%-18s %-12.1e %-12.1e %-12.1e\n', 'σ_aw [m/s²√s]', ...
                    mems.sigma_a_w, tact.sigma_a_w, nav.sigma_a_w);
            fprintf('%-18s %-12.1e %-12.1e %-12.1e\n', 'σ_ωw [rad/s√s]', ...
                    mems.sigma_omega_w, tact.sigma_omega_w, nav.sigma_omega_w);
            fprintf('========================================\n\n');
        end
    end
end
