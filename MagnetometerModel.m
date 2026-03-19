classdef MagnetometerModel < handle
    % MAGNETOMETERMODEL Realistic Magnetometer sensor model with noise and bias random walk
    %
    % Notation follows IMUModel convention:
    %   σ_mn [T or normalized]  : Magnetometer measurement noise std
    %   σ_mw [T√s or √s]        : Magnetometer bias random walk
    %
    % Discrete noise covariance:
    %   M_i = σ²_mw * Δt * I₃   - mag bias RW covariance
    %
    % Measurement model:
    %   mag_meas = R_b2e' * B_ned - bias_mag + noise
    %
    % Bias random walk:
    %   bias_mag(k+1) = bias_mag(k) + m_i,  where m_i ~ N(0, M_i)
    
    properties
        % Sample rate
        fs              % Sample frequency [Hz]
        dt              % Sample period Δt [s]
        
        % Noise parameters
        sigma_mag_n     % σ_mn: Magnetometer measurement noise std
        sigma_mag_w     % σ_mw: Magnetometer bias random walk
        
        % Discrete noise covariance
        M_i             % Mag bias RW cov: σ²_mw * Δt * I₃
        
        % Current bias state (evolving)
        bias_mag        % Current magnetometer bias (3x1)
        
        % Initial bias value
        bias_mag_init
    end
    
    methods
        function obj = MagnetometerModel(params)
            % Constructor
            %
            % Required params:
            %   .fs             - Sample frequency [Hz]
            %
            % Optional params (with defaults):
            %   .sigma_mag_n    - Mag noise std (default: 0.01)
            %   .sigma_mag_w    - Mag bias RW (default: 1e-5)
            %   .bias_mag_init  - Initial mag bias (default: zeros)
            
            % Sample rate (required)
            obj.fs = params.fs;
            obj.dt = 1 / obj.fs;
            
            % === Noise parameters with defaults ===
            if isfield(params, 'sigma_mag_n')
                obj.sigma_mag_n = params.sigma_mag_n;
            else
                obj.sigma_mag_n = 0.01;  % Typical magnetometer noise
            end
            
            if isfield(params, 'sigma_mag_w')
                obj.sigma_mag_w = params.sigma_mag_w;
            else
                obj.sigma_mag_w = 1e-5;  % Typical bias random walk
            end
            
            % === Initial bias ===
            if isfield(params, 'bias_mag_init')
                obj.bias_mag_init = params.bias_mag_init(:);
            else
                obj.bias_mag_init = zeros(3, 1);
            end
            
            % Initialize current bias
            obj.bias_mag = obj.bias_mag_init;
            
            % Compute discrete covariance
            obj.computeDiscreteCovariances(obj.dt);
            
            % Print summary
            obj.printSummary();
        end
        
        function computeDiscreteCovariances(obj, dt_eskf)
            % Compute discrete noise covariance:
            %   M_i = σ²_mw * Δt * I₃
            obj.M_i = obj.sigma_mag_w^2 * dt_eskf * eye(3);
        end
        
        function [mag_meas, bias_mag_out] = measure(obj, R_b2e, B_ned)
            % Generate noisy magnetometer measurement with evolving bias
            %
            % Inputs:
            %   R_b2e  - Rotation matrix from body to earth frame (3x3)
            %   B_ned  - Magnetic field in NED frame (3x1)
            %
            % Outputs:
            %   mag_meas     - Measured magnetic field in body frame (3x1)
            %   bias_mag_out - Current magnetometer bias (3x1)
            %
            % Model:
            %   mag_meas = R_b2e' * B_ned - bias_mag + noise
            %   (Note: We subtract bias in measurement, so filter estimates what to add back)
            
            % Update bias via random walk:
            %   bias_mag(k+1) = bias_mag(k) + m_i, where m_i ~ N(0, M_i)
            m_i = sqrt(obj.M_i(1,1)) * randn(3, 1);
            obj.bias_mag = obj.bias_mag + m_i;
            
            % True magnetic field in body frame
            mag_true = R_b2e' * B_ned;
            
            % Generate measurement noise: n_m ~ N(0, σ²_mn * I)
            n_mag = obj.sigma_mag_n * randn(3, 1);
            
            % Measured value = true - bias + noise
            % (The filter will estimate bias to add back)
            mag_meas = mag_true - obj.bias_mag + n_mag;
            
            % Output current bias
            bias_mag_out = obj.bias_mag;
        end
        
        function reset(obj)
            % Reset bias to initial value
            obj.bias_mag = obj.bias_mag_init;
        end
        
        function M_i = getDiscreteProcessNoise(obj, dt_eskf)
            % Get discrete-time process noise covariance M_i (3x3)
            if nargin > 1
                obj.computeDiscreteCovariances(dt_eskf);
            end
            M_i = obj.M_i;
        end
        
        function sigma_sq = getContinuousProcessNoise(obj)
            % Get continuous-time process noise variance σ²_mw
            % This is for extending Qc in the ESKF
            sigma_sq = obj.sigma_mag_w^2;
        end
        
        function printSummary(obj)
            % Print magnetometer model configuration
            fprintf('\n======== Magnetometer Model Configuration ========\n');
            fprintf('Sample Rate: %.0f Hz (Δt = %.4f s)\n', obj.fs, obj.dt);
            fprintf('\n--- Noise Parameters ---\n');
            fprintf('σ_mn (mag noise):         %.4e\n', obj.sigma_mag_n);
            fprintf('σ_mw (mag bias RW):       %.4e √s\n', obj.sigma_mag_w);
            fprintf('\n--- Discrete Covariance (at %.0f Hz) ---\n', obj.fs);
            fprintf('M_i = σ²_mw·Δt·I:    diag = %.4e\n', obj.M_i(1,1));
            fprintf('\n--- Initial Bias ---\n');
            fprintf('b_mag(0): [%.4f, %.4f, %.4f]\n', obj.bias_mag_init');
            fprintf('==================================================\n\n');
        end
    end
    
    methods (Static)
        function params = typicalMEMS()
            % Return typical MEMS magnetometer parameters (HMC5883L class)
            params = struct();
            params.sigma_mag_n = 0.01;       % Measurement noise
            params.sigma_mag_w = 1e-5;       % Bias random walk
            params.bias_mag_init = [0.001; -0.002; 0.001];
        end
        
        function params = highPerformance()
            % Return high-performance magnetometer parameters
            params = struct();
            params.sigma_mag_n = 0.001;      % Lower noise
            params.sigma_mag_w = 1e-6;       % Slower bias drift
            params.bias_mag_init = zeros(3, 1);
        end
    end
end
