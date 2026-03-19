classdef RadarEmulator < handle
    % RADAREMULATOR  Physically-rigorous radar sensor simulator for UAV navigation.
    %
    % Models radar measurement errors in the radar's natural spherical
    % (range, azimuth, elevation) space rather than in Cartesian space.
    % This correctly reproduces the elongated, cone-shaped error volumes
    % seen in real radar hardware.
    %
    % -----------------------------------------------------------------------
    % NOISE MODEL (spherical domain)
    % -----------------------------------------------------------------------
    %   Noisy Range     : r_m     = r     + N(0, rmse_range^2)
    %   Noisy Azimuth   : theta_m = theta + N(0, rmse_azimuth^2)
    %   Noisy Elevation : phi_m   = phi   + N(0, rmse_elevation^2)
    %   Noisy Doppler   : rdot_m  = rdot  + N(0, rmse_doppler^2)
    %
    % Angular rate noise from finite-differencing of angular positions:
    %   thetadot_m = thetadot + N(0, (rmse_azimuth/dt)^2)
    %   phidot_m   = phidot   + N(0, (rmse_elevation/dt)^2)
    %
    % -----------------------------------------------------------------------
    % COORDINATE CONVENTION (NED)
    % -----------------------------------------------------------------------
    %   NED : x = North, y = East, z = Down
    %   Azimuth   theta = atan2(y, x)          [rad]  — clockwise from North
    %   Elevation phi   = atan2(-z, sqrt(x^2+y^2)) [rad]  — positive UP
    %
    % -----------------------------------------------------------------------
    % REQUIREMENTS
    % -----------------------------------------------------------------------
    %   Requires Matlab Mapping Toolbox for geodetic2ned, ned2ecef, ecef2ned.
    %
    % -----------------------------------------------------------------------
    % USAGE
    % -----------------------------------------------------------------------
    %   params.rmse_range     = 3;           % [m]
    %   params.rmse_azimuth   = deg2rad(1.3); % [rad]
    %   params.rmse_elevation = deg2rad(3.3); % [rad]
    %   params.rmse_doppler   = 0.5;          % [m/s]
    %   radar = RadarEmulator(params);
    %
    %   params.radar_lla   = [39.9; 32.9; 0];   % radar geodetic loc  [lat,lon,alt deg/m]
    %   params.ref_lla0    = [39.9; 32.9; 0];   % NED origin          [lat,lon,alt deg/m]
    %   params.dt          = 2.0;               % measurement period  [s]
    %   radar = RadarEmulator(params);
    %
    %   [pos_ned, vel_ned] = radar.emulate_measurement(target_lla, target_vel_ned);
    %
    % See also: geodetic2ned, ned2ecef, ecef2ned
    
    % =====================================================================
    properties
        % ---- Spherical noise parameters (standard deviations = RMSE) ----
        rmse_range      % Range RMSE                        [m]
        rmse_azimuth    % Azimuth RMSE                      [rad]
        rmse_elevation  % Elevation RMSE                    [rad]
        rmse_doppler    % Radial velocity (Doppler) RMSE    [m/s]

        % ---- Fixed geometry / timing (set once at construction) ----
        radar_lla       % Radar geodetic location [lat,lon,alt]  (3x1) [deg,deg,m]
        ref_lla0        % NED reference origin    [lat,lon,alt]  (3x1) [deg,deg,m]
        dt              % Measurement period                      [s]
    end
    
    % =====================================================================
    methods
        
        % -----------------------------------------------------------------
        function obj = RadarEmulator(params)
            % RadarEmulator  Constructor.
            %
            % Required params (struct):
            %   .radar_lla      - Radar geodetic location [lat,lon,alt] [deg,deg,m] (3x1)
            %   .ref_lla0       - NED reference origin    [lat,lon,alt] [deg,deg,m] (3x1)
            %   .dt             - Measurement period [s]
            %
            % Optional params (with defaults):
            %   .rmse_range     - Range RMSE [m]         (default: 3)
            %   .rmse_azimuth   - Azimuth RMSE [rad]     (default: deg2rad(1.3))
            %   .rmse_elevation - Elevation RMSE [rad]   (default: deg2rad(3.3))
            %   .rmse_doppler   - Doppler RMSE [m/s]     (default: 0.5)

            if isfield(params, 'rmse_range')
                obj.rmse_range = params.rmse_range;
            else
                obj.rmse_range = 3.0;
            end

            if isfield(params, 'rmse_azimuth')
                obj.rmse_azimuth = params.rmse_azimuth;
            else
                obj.rmse_azimuth = deg2rad(1.3);
            end

            if isfield(params, 'rmse_elevation')
                obj.rmse_elevation = params.rmse_elevation;
            else
                obj.rmse_elevation = deg2rad(3.3);
            end

            if isfield(params, 'rmse_doppler')
                obj.rmse_doppler = params.rmse_doppler;
            else
                obj.rmse_doppler = 0.5;
            end

            % === Fixed geometry / timing ===
            assert(isfield(params, 'radar_lla'), 'RadarEmulator: params.radar_lla is required.');
            assert(isfield(params, 'ref_lla0'),  'RadarEmulator: params.ref_lla0 is required.');
            assert(isfield(params, 'dt'),         'RadarEmulator: params.dt is required.');

            obj.radar_lla = params.radar_lla(:);   % force 3x1
            obj.ref_lla0  = params.ref_lla0(:);    % force 3x1
            obj.dt        = params.dt;

            assert(numel(obj.radar_lla) == 3, 'radar_lla must have 3 elements [lat,lon,alt].');
            assert(numel(obj.ref_lla0)  == 3, 'ref_lla0 must have 3 elements [lat0,lon0,alt0].');
            assert(isscalar(obj.dt) && obj.dt > 0, 'dt must be a positive scalar [s].');

            % === Validate noise parameters ===
            assert(obj.rmse_range     > 0, 'RadarEmulator: rmse_range must be positive.');
            assert(obj.rmse_azimuth   > 0, 'RadarEmulator: rmse_azimuth must be positive (radians).');
            assert(obj.rmse_elevation > 0, 'RadarEmulator: rmse_elevation must be positive (radians).');
            assert(obj.rmse_doppler   > 0, 'RadarEmulator: rmse_doppler must be positive.');

            obj.printSummary();
        end

        % -----------------------------------------------------------------
        function [noisy_pos_ned0, noisy_vel_ned0, R_pos, R_vel] = ...
            emulate_measurement(obj, target_lla, target_vel_ned)
            
            % emulate_measurement  Produce one noisy radar observation.
            %
            % Implements a six-step spherical noise injection pipeline:
            %   1. Ground truth NED position relative to radar
            %   2. Convert to spherical observables (r, θ, φ)
            %   3. Inject Gaussian noise in spherical domain
            %   4. Back-project noisy spherical -> noisy NED_radar
            %   5. Transform noisy position to obj.ref_lla0 NED frame
            %   6. Velocity: decompose into radial/angular rates, inject
            %      noise, reconstruct, rotate to obj.ref_lla0 NED frame
            %
            % ----- Inputs -----
            %   target_lla     - Target geodetic location [lat,lon,alt]  (3x1) [deg,deg,m]
            %   target_vel_ned - Target velocity in target-local NED     (3x1) [m/s]
            %
            % Uses class properties: obj.radar_lla, obj.ref_lla0, obj.dt
            %
            % ----- Outputs -----
            %   noisy_pos_ned0 - Noisy target position in ref_lla0 NED  (3x1) [m]
            %   noisy_vel_ned0 - Noisy target velocity in ref_lla0 NED  (3x1) [m/s]

            % --- Input validation ---
            target_lla     = target_lla(:);
            target_vel_ned = target_vel_ned(:);

            assert(numel(target_lla)     == 3, 'target_lla must have 3 elements [lat,lon,alt].');
            assert(numel(target_vel_ned) == 3, 'target_vel_ned must have 3 elements [vN,vE,vD].');

            % ==============================================================
            % STEP 1: Ground truth Cartesian position in radar's NED frame
            % ==============================================================
            % geodetic2ned(lat,lon,alt, lat0,lon0,alt0, referenceEllipsoid)
            % Returns scalar (xNorth, yEast, zDown) — combine into 3x1 column
            [xN, xE, xD] = geodetic2ned( ...
                target_lla(1), target_lla(2), target_lla(3), ...
                obj.radar_lla(1),  obj.radar_lla(2),  obj.radar_lla(3), ...
                referenceEllipsoid('wgs84'));

            P_true_radar = [xN; xE; xD];  % True relative position [N;E;D] [m]

            % Sanity: the target must not be co-located with the radar
            r_true = norm(P_true_radar);
            assert(r_true > 1e-3, ...
                'RadarEmulator: target and radar are effectively co-located (r < 1 mm).');

            % ==============================================================
            % STEP 2: True spherical observables
            % ==============================================================
            %   x = North, y = East, z = Down  (NED)
            x = P_true_radar(1);   % North
            y = P_true_radar(2);   % East
            z = P_true_radar(3);   % Down

            r_xy = sqrt(x^2 + y^2);   % Horizontal range

            % Range
            r = sqrt(x^2 + y^2 + z^2);

            % Azimuth: angle from North toward East (clockwise in NED top view)
            theta = atan2(y, x);       % [-pi, pi] rad

            % Elevation: positive above horizon
            % Down is +z in NED, so elevation = atan2(-z, r_xy)
            phi = atan2(-z, r_xy);     % [-pi/2, pi/2] rad

            % ==============================================================
            % STEP 3: Inject Gaussian noise in spherical domain
            % ==============================================================
            r_m     = r     + obj.rmse_range     * randn();
            theta_m = theta + obj.rmse_azimuth   * randn();
            phi_m   = phi   + obj.rmse_elevation * randn();

            % Clamp range to physical minimum (must be positive)
            if r_m < 0.01
               warning('There is a problem on radar range measurement!')
            end
            r_m = max(r_m, 0.01);

            % Calculate meas. cov. R(p_ned) based on noisy sph. meas.
            [R_pos, R_vel] = obj.getCartesianCovariance(r_m, theta_m, phi_m);

            % ==============================================================
            % STEP 4: Noisy spherical -> noisy Cartesian in radar's NED frame
            % ==============================================================
            %   x_m =  r_m * cos(phi_m) * cos(theta_m)   (North)
            %   y_m =  r_m * cos(phi_m) * sin(theta_m)   (East)
            %   z_m = -r_m * sin(phi_m)                  (Down, negative because phi up is positive)
            cos_phi_m = cos(phi_m);
            xN_m =  r_m * cos_phi_m * cos(theta_m);
            xE_m =  r_m * cos_phi_m * sin(theta_m);
            xD_m = -r_m * sin(phi_m);

            P_noisy_ned_radar = [xN_m; xE_m; xD_m];  % [m] in radar-local NED

            % ==============================================================
            % STEP 5: Transform noisy NED_radar -> ECEF -> NED at ref_lla0
            % ==============================================================
            % Intermediate: noisy radar-local NED  -> ECEF
            [ecef_x, ecef_y, ecef_z] = ned2ecef( ...
                P_noisy_ned_radar(1), P_noisy_ned_radar(2), P_noisy_ned_radar(3), ...
                obj.radar_lla(1), obj.radar_lla(2), obj.radar_lla(3), ...
                referenceEllipsoid('wgs84'));

            % Final: ECEF -> NED centered at ref_lla0
            [pN_0, pE_0, pD_0] = ecef2ned( ...
                ecef_x, ecef_y, ecef_z, ...
                obj.ref_lla0(1), obj.ref_lla0(2), obj.ref_lla0(3), ...
                referenceEllipsoid('wgs84'));

            noisy_pos_ned0 = [pN_0; pE_0; pD_0];  % 3x1 [m]

            % ==============================================================
            % STEP 6: Realistic velocity emulation
            % ==============================================================
            %
            % Radar measures radial velocity (Doppler) directly and
            % cross-range velocity indirectly from angular tracking rates.
            %
            % Decompose target_vel_ned (in target-local NED ≈ radar NED for
            % near-field; explicit rotation is identity here as both frames
            % share the same NED orientation) into spherical rate components.
            %
            % The NED velocity of the target is expressed in the standard
            % NED basis aligned with the geodetic surface at the target.
            % For short baselines (< ~50 km) the radar's and target's NED
            % frames are virtually co-aligned; we adopt that approximation
            % and note it explicitly:
            %
            %   NOTE (small-baseline approximation): The target's NED frame
            %   and the radar's NED frame are treated as parallel, i.e. the
            %   rotation between them is the identity. For large baselines
            %   this should be replaced with the exact DCM computed from
            %   the geodetic coordinates of both sites.

            vN = target_vel_ned(1);
            vE = target_vel_ned(2);
            vD = target_vel_ned(3);

            % True unit radial vector (NED)
            ur = P_true_radar / r;   % [N;E;D] pointing from radar to target

            % --- True radial velocity (Doppler) ---
            rdot = dot([vN; vE; vD], ur);   % projection onto LOS [m/s]

            % --- True angular rates ---
            % From differential of spherical coordinates:
            %   theta = atan2(y, x)  =>  thetadot = (x*ydot - y*xdot) / (x^2+y^2)
            %   phi   = atan2(-z, sqrt(x^2+y^2))
            %         =>  phidot = (r_xy * (-zdot) - (-z)*r_dot_xy) / (r_xy * r)
            %                    = (r_xy * (-vD) - (-z)*rdot_xy) / (r_xy * r)
            %   where  rdot_xy = (x*vN + y*vE) / r_xy

            r_xy_true = sqrt(x^2 + y^2);

            if r_xy_true < 1e-6
                % Target is directly above/below radar — azimuth rate ill-defined
                thetadot = 0;
                phidot   = 0;
                warning('Target directly above/below of radar')
            else
                % Azimuth rate
                thetadot = (x * vE - y * vN) / (r_xy_true^2);   % [rad/s]

                % Elevation rate
                rdot_xy  = (x * vN + y * vE) / r_xy_true;        % horizontal range rate
                phidot   = (r_xy_true * (-vD) - (-z) * rdot_xy) / (r_xy_true * r); % [rad/s]
            end

            % --- Inject noise ---
            rdot_m     = rdot     + obj.rmse_doppler                  * randn();
            thetadot_m = thetadot + (obj.rmse_azimuth   / obj.dt)         * randn();
            phidot_m   = phidot   + (obj.rmse_elevation / obj.dt)         * randn();

            % --- Reconstruct NED velocity from noisy spherical rates ---
            % From inverse Jacobian of spherical -> Cartesian:
            %   vN =  rdot*cos(phi)*cos(theta)
            %       - r*phidot*sin(phi)*cos(theta)
            %       - r*thetadot*cos(phi)*sin(theta)
            %
            %   vE =  rdot*cos(phi)*sin(theta)
            %       - r*phidot*sin(phi)*sin(theta)
            %       + r*thetadot*cos(phi)*cos(theta)
            %
            %   vD = -rdot*sin(phi)
            %       - r*phidot*cos(phi)
            %
            % Evaluate using NOISY spherical observables (r_m, theta_m, phi_m)
            % for consistency with the noisy position output.

            cp_m = cos(phi_m);
            sp_m = sin(phi_m);
            ct_m = cos(theta_m);
            st_m = sin(theta_m);

            vN_m =  rdot_m * cp_m * ct_m ...
                  - r_m * phidot_m   * sp_m * ct_m ...
                  - r_m * thetadot_m * cp_m * st_m;

            vE_m =  rdot_m * cp_m * st_m ...
                  - r_m * phidot_m   * sp_m * st_m ...
                  + r_m * thetadot_m * cp_m * ct_m;

            vD_m = -rdot_m * sp_m ...
                  - r_m * phidot_m   * cp_m;

            % NOTE (NED frame rotation): The velocity vector is now in the
            % radar's local NED frame. To express it in the ref_lla0 NED
            % frame exactly, we would apply the DCM between the two sites.
            % Under the small-baseline approximation adopted above, that
            % DCM is the identity, so no further rotation is applied.
            noisy_vel_ned0 = [vN_m; vE_m; vD_m];  % 3x1 [m/s]
        end

        % -----------------------------------------------------------------
        function [R_pos, R_vel] = getCartesianCovariance(obj, r, theta, phi)
            % getCartesianCovariance  Compute anisotropic Cartesian covariance
            %   from spherical RMSE values via first-order error propagation.
            %
            % The spherical-to-NED mapping is:
            %   N =  r * cos(phi) * cos(theta)
            %   E =  r * cos(phi) * sin(theta)
            %   D = -r * sin(phi)
            %
            % Jacobian  J = d[N,E,D]' / d[r, theta, phi]'
            %
            %   J = [ cos(phi)*cos(theta),  -r*cos(phi)*sin(theta),  -r*sin(phi)*cos(theta) ]
            %       [ cos(phi)*sin(theta),   r*cos(phi)*cos(theta),  -r*sin(phi)*sin(theta) ]
            %       [-sin(phi),              0,                       -r*cos(phi)            ]
            %
            % Position covariance (first-order):
            %   Sigma_sph = diag([sigma_r^2, sigma_az^2, sigma_el^2])
            %   R_pos = J * Sigma_sph * J'
            %
            % Velocity covariance (same Jacobian, different diagonal noise):
            %   Sigma_vel = diag([sigma_rdot^2, (sigma_az/dt)^2, (sigma_el/dt)^2])
            %   R_vel = J * Sigma_vel * J'
            %
            % Inputs:
            %   r     - Slant range to target             [m]
            %   theta - Azimuth angle                     [rad]
            %   phi   - Elevation angle (positive up)     [rad]
            %
            % Outputs:
            %   R_pos - 3x3 Cartesian position noise covariance  [m^2]
            %   R_vel - 3x3 Cartesian velocity noise covariance  [m^2/s^2]

            cp = cos(phi);   sp = sin(phi);
            ct = cos(theta); st = sin(theta);

            % Spherical -> NED Jacobian (3x3)
            J = [ cp*ct,   -r*cp*st,   -r*sp*ct;
                  cp*st,    r*cp*ct,   -r*sp*st;
                 -sp,       0,         -r*cp   ];

            % Spherical noise covariance (position domain)
            S_pos = diag([obj.rmse_range^2, ...
                          obj.rmse_azimuth^2, ...
                          obj.rmse_elevation^2]);

            % Spherical noise covariance (velocity domain — angular noise / dt)
            S_vel = diag([obj.rmse_doppler^2, ...
                          (obj.rmse_azimuth   / obj.dt)^2, ...
                          (obj.rmse_elevation / obj.dt)^2]);

            R_pos = J * S_pos * J';
            R_vel = J * S_vel * J';

            % Symmetrize to guard against floating-point asymmetry
            R_pos = 0.5 * (R_pos + R_pos');
            R_vel = 0.5 * (R_vel + R_vel');
        end

        % -----------------------------------------------------------------
        function printSummary(obj)
            % Print radar emulator configuration summary
            fprintf('\n======== RadarEmulator Configuration ========\n');
            fprintf('--- Geometry & Timing ---\n');
            fprintf('  radar_lla  : [%.4f deg, %.4f deg, %.1f m]\n', ...
                    obj.radar_lla(1), obj.radar_lla(2), obj.radar_lla(3));
            fprintf('  ref_lla0   : [%.4f deg, %.4f deg, %.1f m]\n', ...
                    obj.ref_lla0(1),  obj.ref_lla0(2),  obj.ref_lla0(3));
            fprintf('  dt         : %.4f s\n', obj.dt);
            fprintf('--- Spherical Noise Parameters ---\n');
            fprintf('  rmse_range     : %.4f m\n',     obj.rmse_range);
            fprintf('  rmse_azimuth   : %.4f rad  (%.4f deg)\n', ...
                    obj.rmse_azimuth, rad2deg(obj.rmse_azimuth));
            fprintf('  rmse_elevation : %.4f rad  (%.4f deg)\n', ...
                    obj.rmse_elevation, rad2deg(obj.rmse_elevation));
            fprintf('  rmse_doppler   : %.4f m/s\n',   obj.rmse_doppler);
            fprintf('==============================================\n\n');
        end

    end  % methods

    % =====================================================================
    methods (Static)

        function params = defaultUAV()
            % defaultUAV  Typical airborne surveillance radar for UAV tracking.
            %
            % Based on representative values for a short-range X-band radar:
            %   Range RMSE       : 5 m
            %   Azimuth RMSE     : 0.1 deg
            %   Elevation RMSE   : 0.1 deg
            %   Doppler RMSE     : 0.1 m/s
            params = struct();
            params.rmse_range     = 5.0;
            params.rmse_azimuth   = deg2rad(0.1);
            params.rmse_elevation = deg2rad(0.1);
            params.rmse_doppler   = 0.1;
        end

        function params = highAccuracy()
            % highAccuracy  High-precision tracking radar (e.g. AESA).
            params = struct();
            params.rmse_range     = 1.0;
            params.rmse_azimuth   = deg2rad(0.02);
            params.rmse_elevation = deg2rad(0.02);
            params.rmse_doppler   = 0.05;
        end

        function params = longRange()
            % longRange  Long-range surveillance radar (coarser angular resolution).
            params = struct();
            params.rmse_range     = 50.0;
            params.rmse_azimuth   = deg2rad(0.5);
            params.rmse_elevation = deg2rad(0.5);
            params.rmse_doppler   = 0.5;
        end

    end  % methods (Static)

end  % classdef RadarEmulator
