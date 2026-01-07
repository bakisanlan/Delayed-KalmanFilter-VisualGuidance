classdef ESKFSimulation < handle
    % ESKFSimulation Encapsulates the ESKF Closed-Loop simulation for GUI usage
    
    properties
        % --- Components ---
        imu             % IMUModel instance
        eskf            % ErrorStateKalmanFilter instance
        ctrl            % InterceptorController instance
        
        % --- Configuration ---
        config          % Structure containing all settings
        
        % --- State ---
        t               % Current simulation time
        k               % Current step index
        x_true          % Current true state (18x1)
        x_est           % Current estimated state (18x1)
        P               % Current covariance (17x17)
        p_int           % Interceptor position (NED)
        v_int           % Interceptor velocity
        p_tgt           % Target position
        v_tgt           % Target velocity
        
        % --- History ---
        history         % Struct to store simulation history for plotting
        
        % --- Counters & Buffers ---
        eskf_update_counter
        image_update_counter
        radar_update_counter
        omega_accum
        a_accum
        imu_count
        
        % --- Constants ---
        g = 9.81;
        e3 = [0; 0; 1];
    end
    
    methods
        function obj = ESKFSimulation()
            % Constructor - Set default config or empty
            obj.t = 0;
            obj.k = 0;
            obj.config = obj.getDefaultConfig();
        end
        
        function initialize(obj, userConfig)
            % Initialize simulation with user configuration
            if nargin > 1
                % Merge user config with defaults (shallow)
                fields = fieldnames(userConfig);
                for i = 1:numel(fields)
                    obj.config.(fields{i}) = userConfig.(fields{i});
                end
            end
            
            cfg = obj.config;
            
            % 1. IMU Model
            imu_params = struct();
            imu_params.fs = 1/cfg.dt_imu;
            imu_params.sigma_a_n = cfg.imu_noise.accel_n;
            imu_params.sigma_omega_n = cfg.imu_noise.gyro_n;
            imu_params.sigma_a_w = cfg.imu_noise.accel_w;
            imu_params.sigma_omega_w = cfg.imu_noise.gyro_w;
            imu_params.omega_b_init = [0.005; -0.003; 0.002]; 
            imu_params.a_b_init     = [0.02; -0.01; 0.015];
            obj.imu = IMUModel(imu_params);
            
            % 2. Controller
            ctrl_params = struct();
            ctrl_params.Kp_att = 5;
            ctrl_params.Kp_acc = 1;
            % Use GUI-specified constraints if available
            if isfield(cfg, 'controller') && isfield(cfg.controller, 'max_omega')
                ctrl_params.max_omega = cfg.controller.max_omega;
            end
            if isfield(cfg, 'controller') && isfield(cfg.controller, 'max_velocity')
                ctrl_params.max_acc = cfg.controller.max_velocity; % Use as max_acc for now
            end
            obj.ctrl = InterceptorController(ctrl_params);
            
            % 3. ErrorStateKalmanFilter
            % Initial True State
            obj.p_int = cfg.init_cond.p_int;
            obj.v_int = cfg.init_cond.v_int;
            obj.p_tgt = cfg.init_cond.p_tgt;
            obj.v_tgt = cfg.init_cond.v_tgt;
            
            p_r_true = obj.p_int - obj.p_tgt;
            v_r_true = obj.v_int - obj.v_tgt;
            
             % Camera Rotation
            R_c2b = [0 0 1; 1 0 0; 0 1 0];
            R_b2c = R_c2b';
            
            % Initial Attitude (from config or default)
            if isfield(cfg.init_cond, 'euler_int') && ~isempty(cfg.init_cond.euler_int)
                euler_init = cfg.init_cond.euler_int; % Already in radians from GUI
            else
                euler_init = deg2rad([45, 20, 15]); % Default [Yaw, Pitch, Roll]
            end
            q_true = eul2quat(euler_init, 'ZYX')';
            
            % Initial Image Features
            pbar_true = compute_image_features(p_r_true, q_true, R_b2c);
            
            % Biases
            omega_b_true = obj.imu.omega_b;
            a_b_true = obj.imu.a_b;
            
            obj.x_true = [q_true; p_r_true; v_r_true; pbar_true; omega_b_true; a_b_true];
            
            % Filter Initialization
            % Create initial state with some error
            x_init = ErrorStateKalmanFilter.createInitialState(q_true, p_r_true, v_r_true, ...
                                                                pbar_true, omega_b_true, a_b_true, ...
                                                                cfg.filter.init_errors);
            
            P_init = ErrorStateKalmanFilter.createInitialCovariance(cfg.filter.init_sigma);
            
            eskf_params = struct();
            eskf_params.dt_imu = cfg.dt_imu;
            eskf_params.dt_eskf = cfg.dt_eskf;
            eskf_params.R_b2c = R_b2c;
            eskf_params.Qd = obj.imu.getDiscreteProcessNoise(cfg.dt_eskf);
            eskf_params.Qc = obj.imu.getESKFProcessNoise();
            eskf_params.R_img = cfg.sensors.sigma_img^2 * eye(2);
            eskf_params.R_radar = cfg.sensors.sigma_radar^2 * eye(3);
            eskf_params.x_init = x_init;
            eskf_params.P_init = P_init;
            eskf_params.delay_steps = round(cfg.sensors.delay / cfg.dt_imu);
            
            obj.eskf = ErrorStateKalmanFilter(eskf_params);
            obj.P = P_init;
            obj.x_est = x_init;
            
            % 4. Initialize History
            est_steps = ceil(cfg.t_total / cfg.dt_imu);
            obj.history = struct();
            obj.history.t = zeros(1, est_steps);
            obj.history.x_true = zeros(18, est_steps);
            obj.history.x_est = zeros(18, est_steps);
            obj.history.P_diag = zeros(17, est_steps);
            obj.history.omega_cmd = zeros(3, est_steps);
            obj.history.a_cmd = zeros(3, est_steps);
            obj.history.p_int = zeros(3, est_steps); % Absolute interceptor pos
            obj.history.p_tgt = zeros(3, est_steps); % Absolute target pos
            obj.history.count = 0;
            
            % 5. Reset Counters
            obj.t = 0;
            obj.k = 0;
            obj.eskf_update_counter = 0;
            obj.image_update_counter = 0;
            obj.radar_update_counter = 0;
            obj.omega_accum = zeros(3, 1);
            obj.a_accum = zeros(3, 1);
            obj.imu_count = 0;
        end
        
        function finished = step(obj)
            % Execute one simulation step (IMU dt)
            % Returns true if simulation finished (time or impact)
            
            cfg = obj.config;
            obj.k = obj.k + 1;
            obj.t = obj.t + cfg.dt_imu;
            
            % --- Indices ---
            idx_q = 1:4; idx_pr = 5:7; idx_vr = 8:10;
            idx_pbar = 11:12; idx_bgyr = 13:15; idx_bacc = 16:18;
            
            R_c2b = [0 0 1; 1 0 0; 0 1 0];
            R_b2c = R_c2b';
            
            % --- Controller ---
            q_est = obj.eskf.x(idx_q);
            p_r_est = obj.eskf.x(idx_pr);
            v_r_est = obj.eskf.x(idx_vr);
            
            [omega_cmd, a_cmd] = obj.ctrl.compute_control(q_est, p_r_est, v_r_est);
            
            % --- Propagate True State ---
            [obj.x_true, obj.p_int, obj.v_int, obj.p_tgt, obj.v_tgt, omega_true, a_body_true] = ...
                propagate_true_state_controlled(obj.x_true, obj.p_int, obj.v_int, obj.p_tgt, obj.v_tgt, ...
                                     obj.t, cfg.dt_imu, obj.g, obj.e3, R_b2c, ...
                                     omega_cmd, a_cmd, ...
                                     idx_q, idx_pr, idx_vr, idx_pbar, idx_bgyr, idx_bacc);
                                 
            % IMU Meas
            [omega_meas, a_meas, omega_b_true, a_b_true] = obj.imu.measure(omega_true, a_body_true);
            obj.x_true(idx_bgyr) = omega_b_true;
            obj.x_true(idx_bacc) = a_b_true;
            
            % --- ESKF Prediction ---
            obj.omega_accum = obj.omega_accum + omega_meas;
            obj.a_accum = obj.a_accum + a_meas;
            obj.imu_count = obj.imu_count + 1;
            
            obj.eskf_update_counter = obj.eskf_update_counter + 1;
            eskf_sample_idx = round(cfg.dt_eskf / cfg.dt_imu);
            
            if obj.eskf_update_counter >= eskf_sample_idx
                obj.eskf_update_counter = 0;
                omega_avg = obj.omega_accum / obj.imu_count;
                a_avg = obj.a_accum / obj.imu_count;
                obj.eskf.predict(omega_avg, a_avg, obj.t);
                obj.omega_accum = zeros(3,1);
                obj.a_accum = zeros(3,1);
                obj.imu_count = 0;
            end
            
            % --- Updates ---
            % Image
            obj.image_update_counter = obj.image_update_counter + 1;
            img_sample_idx = round(cfg.dt_image / cfg.dt_imu);
            delay_steps = round(cfg.sensors.delay / cfg.dt_imu);
            
            if obj.image_update_counter >= img_sample_idx && obj.k > delay_steps
                 obj.image_update_counter = 0;
                 if cfg.sensors.useCam
                     R_b2e_delayed = quat2rotm(obj.x_true(idx_q)'); % Simplified: Using current for delay calc approx
                     % Ideally track history for exact delay. For now, using current approx
                     % In run_eskf_closed_loop it uses x_true direct, which handles delay implicitly by index if logged
                     % Here we might simply use current x_true if delay is small, or implement a buffer.
                     % The original code used x_true at current time for 'delayed' measurement generation 
                     % but applied it with 'D' delay in ESKF.
                     
                     p_r_cam = R_b2c * R_b2e_delayed' * (-obj.x_true(idx_pr));
                     if p_r_cam(3) > 1
                         z_meas = [p_r_cam(1)/p_r_cam(3); p_r_cam(2)/p_r_cam(3)] + cfg.sensors.sigma_img * randn(2,1);
                         obj.eskf.correctImage(z_meas, delay_steps);
                     end
                 end
            end
            
            % Radar
            obj.radar_update_counter = obj.radar_update_counter + 1;
            radar_sample_idx = round(cfg.dt_radar / cfg.dt_imu);
            if cfg.sensors.useRadar && obj.radar_update_counter >= radar_sample_idx
                obj.radar_update_counter = 0;
                z_radar = obj.x_true(idx_pr) + cfg.sensors.sigma_radar * randn(3,1);
                obj.eskf.correctRadar(z_radar);
            end
            
            % --- Update State Properties ---
            obj.x_est = obj.eskf.x;
            obj.P = obj.eskf.P;
            
            % --- Store History ---
            idx = obj.history.count + 1;
            obj.history.t(idx) = obj.t;
            obj.history.x_true(:, idx) = obj.x_true;
            obj.history.x_est(:, idx) = obj.x_est;
            obj.history.P_diag(:, idx) = diag(obj.P);
            obj.history.omega_cmd(:, idx) = omega_cmd;
            obj.history.a_cmd(:, idx) = a_cmd;
            obj.history.p_int(:, idx) = obj.p_int;
            obj.history.p_tgt(:, idx) = obj.p_tgt;
            obj.history.count = idx;
            
            % --- Check Termination ---
            finished = false;
            if norm(obj.x_true(idx_pr)) < 5.0
                fprintf('Impact detected! Dist: %.2f\n', norm(obj.x_true(idx_pr)));
                finished = true;
            elseif obj.t >= cfg.t_total
                finished = true;
            end
            
            % Trim history if finished
            if finished
                obj.trimHistory();
            end
        end
        
        function trimHistory(obj)
             c = obj.history.count;
             fields = fieldnames(obj.history);
             for i = 1:numel(fields)
                 val = obj.history.(fields{i});
                 if numel(val) > 1 && size(val,2) >= c
                     obj.history.(fields{i}) = val(:, 1:c);
                 end
             end
        end
        
        function cfg = getDefaultConfig(~)
            cfg = struct();
            cfg.dt_imu = 1/200;
            cfg.dt_eskf = 1/100;
            cfg.dt_image = 1/30;
            cfg.dt_radar = 1/0.5;
            cfg.t_total = 30;
            
            cfg.imu_noise.accel_n = 0.1;
            cfg.imu_noise.gyro_n = 0.01;
            cfg.imu_noise.accel_w = 1e-4;
            cfg.imu_noise.gyro_w = 1e-5;
            
            cfg.sensors.useCam = true;
            cfg.sensors.useRadar = true;
            cfg.sensors.sigma_img = 0.005;
            cfg.sensors.sigma_radar = 1.0;
            cfg.sensors.delay = 0;
            
            cfg.init_cond.p_int = [0; 0; -65];
            cfg.init_cond.v_int = [5; 0; 0];
            cfg.init_cond.p_tgt = [200; 200; -100];
            cfg.init_cond.v_tgt = [0; 0; 0];
            
            % Default filter uncertainties (Medium)
            cfg.filter.init_errors.euler_deg = [1; 1; 1];
            cfg.filter.init_errors.position = [0.5; 0.5; 0.5];
            cfg.filter.init_errors.velocity = [0.1; -0.05; 0];
            cfg.filter.init_errors.pbar = [0.01; -0.02];
            cfg.filter.init_errors.b_gyr = [0;0;0];
            cfg.filter.init_errors.b_acc = [0;0;0];
            
            cfg.filter.init_sigma.attitude = 0.05;
            cfg.filter.init_sigma.position = 3;
            cfg.filter.init_sigma.velocity = 0.5;
            cfg.filter.init_sigma.pbar = 0.01;
            cfg.filter.init_sigma.b_gyr = 0.005;
            cfg.filter.init_sigma.b_acc = 0.05;
        end
    end
end
