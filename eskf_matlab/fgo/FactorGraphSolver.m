classdef FactorGraphSolver < handle
    % FACTORGRAPHSOLVER  Batch Gauss-Newton factor graph optimizer.
    %
    % Solves nonlinear least-squares problems of the form:
    %
    %   X* = argmin  sum_i  || r_i(X) ||^2_{Sigma_i}
    %
    % where r_i are residual functions (factors) and Sigma_i are noise
    % covariance matrices.  The state X is a collection of N variable nodes,
    % each of dimension state_dim.
    %
    % Supported factor types:
    %   'prior'    - prior on a single variable
    %   'dynamics' - constant-velocity between factor
    %   'camera'   - perspective projection measurement
    %   'radar'    - direct position + velocity measurement
    %
    % Usage:
    %   solver = FactorGraphSolver(N, 6);
    %   solver.setInitialEstimate(1, x0);
    %   solver.addPriorFactor(1, x0_mean, P0);
    %   solver.addDynamicsFactor(1, 2, dt, Sigma_dyn);
    %   solver.addCameraFactor(3, z_img, R_img, R_b2c, R_b2e, p_i);
    %   solver.addRadarFactor(5, z_rad, R_rad);
    %   cost_hist = solver.optimize(30, 1e-6);

    properties
        n_vars        % Number of variable nodes
        state_dim     % Dimension per variable (e.g. 6)
        x             % Current estimates: state_dim x n_vars
        factors       % Cell array of factor structs
        n_factors     % Current number of factors
    end

    methods

        % ==============================================================
        %  CONSTRUCTOR
        % ==============================================================
        function obj = FactorGraphSolver(n_vars, state_dim)
            obj.n_vars    = n_vars;
            obj.state_dim = state_dim;
            obj.x         = zeros(state_dim, n_vars);
            obj.factors   = {};
            obj.n_factors = 0;
        end

        % ==============================================================
        %  INITIAL ESTIMATES
        % ==============================================================
        function setInitialEstimate(obj, key, x_init)
            % Set initial value for variable node 'key'.
            obj.x(:, key) = x_init(:);
        end

        % ==============================================================
        %  FACTOR ADDITION
        % ==============================================================
        function addPriorFactor(obj, key, mean, cov)
            % Add a Gaussian prior  N(mean, cov)  on variable 'key'.
            %   r = x_k - mean
            %   J = I
            f.type      = 'prior';
            f.keys      = key;
            f.z         = mean(:);
            f.sqrt_info = computeSqrtInfo(cov);
            obj.n_factors = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        function addDynamicsFactor(obj, key_k, key_kp1, dt, cov)
            % Constant-velocity between factor.
            %   f(x_k) = [p_t + v_t*dt;  v_t]
            %   r = x_{k+1} - f(x_k)
            f.type      = 'dynamics';
            f.keys      = [key_k, key_kp1];
            f.dt        = dt;
            f.sqrt_info = computeSqrtInfo(cov);
            obj.n_factors = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        function addCameraFactor(obj, key, z_meas, cov, R_b2c, R_b2e, p_i)
            % Perspective-projection camera factor.
            %   h(x_k) = pi( R_b2c * R_b2e' * (p_t - p_i) )
            %   r = z - h(x_k)
            f.type      = 'camera';
            f.keys      = key;
            f.z         = z_meas(:);
            f.sqrt_info = computeSqrtInfo(cov);
            f.R_b2c     = R_b2c;
            f.R_b2e     = R_b2e;
            f.p_i       = p_i(:);
            obj.n_factors = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        function addRadarFactor(obj, key, z_radar, cov)
            % Direct position + velocity observation.
            %   h(x_k) = [p_t; v_t] = x_k
            %   r = z - x_k
            f.type      = 'radar';
            f.keys      = key;
            f.z         = z_radar(:);
            f.sqrt_info = computeSqrtInfo(cov);
            obj.n_factors = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        % ==============================================================
        %  GAUSS-NEWTON OPTIMISATION
        % ==============================================================
        function cost_history = optimize(obj, max_iter, tol)
            % Run iterative Gauss-Newton optimisation.
            %
            %  1. Evaluate all factors  ->  r_i, J_i
            %  2. Build  H = J' J,  b = -J' r   (whitened)
            %  3. Solve  H dx = b   (sparse Cholesky)
            %  4. Update  x <- x + dx
            %  5. Repeat until  ||dx|| < tol

            cost_history = zeros(max_iter, 1);

            for iter = 1:max_iter
                % Build sparse whitened Jacobian and residual
                [J, r] = obj.buildWhitenedSystem();

                cost = 0.5 * (r' * r);
                cost_history(iter) = cost;

                % Normal equations
                H = J' * J;
                b = -J' * r;

                % Add small LM-style damping for robustness
                lambda = 1e-8;
                H = H + lambda * speye(size(H));

                % Solve
                dx = H \ b;

                % Update all states
                for k = 1:obj.n_vars
                    idx = (k-1)*obj.state_dim + (1:obj.state_dim);
                    obj.x(:, k) = obj.x(:, k) + dx(idx);
                end

                dx_norm = norm(dx);
                fprintf('  GN iter %2d:  cost = %10.4e   ||dx|| = %8.2e\n', ...
                        iter, cost, dx_norm);

                % Convergence check
                if dx_norm < tol
                    cost_history = cost_history(1:iter);
                    fprintf('  Converged in %d iterations.\n', iter);
                    return
                end
            end

            fprintf('  WARNING: did not converge in %d iterations.\n', max_iter);
            cost_history = cost_history(1:max_iter);
        end

        % ==============================================================
        %  MARGINAL COVARIANCE
        % ==============================================================
        function Sigma_k = computeMarginalCovariance(obj, key)
            % Compute the marginal covariance for variable node 'key'
            % from the Hessian approximation  H = J' * J.
            %
            %   Sigma_k = [H^{-1}]_{kk}
            %
            % Uses sparse back-substitution (not full inverse).

            [J, ~] = obj.buildWhitenedSystem();
            H = J' * J;
            H = H + 1e-10 * speye(size(H));   % regularise

            idx = (key-1)*obj.state_dim + (1:obj.state_dim);
            n = size(H, 1);
            E = sparse(idx, 1:obj.state_dim, ...
                       ones(obj.state_dim, 1), n, obj.state_dim);
            C = H \ E;
            Sigma_k = full(C(idx, :));
            Sigma_k = 0.5 * (Sigma_k + Sigma_k');   % symmetrise
        end

        function P_all = computeAllMarginals(obj)
            % Compute marginal covariance for ALL variable nodes.
            % Returns state_dim x state_dim x n_vars array.

            [J, ~] = obj.buildWhitenedSystem();
            H = J' * J;
            H = H + 1e-10 * speye(size(H));   % regularise

            n = size(H, 1);
            % Solve H * C = I  (full inverse — feasible for moderate size)
            H_inv = inv(H);  %#ok  — acceptable for <3000 unknowns

            P_all = zeros(obj.state_dim, obj.state_dim, obj.n_vars);
            for k = 1:obj.n_vars
                idx = (k-1)*obj.state_dim + (1:obj.state_dim);
                Pk = H_inv(idx, idx);
                P_all(:,:,k) = 0.5 * (Pk + Pk');
            end
        end

        % ==============================================================
        %  GETTERS
        % ==============================================================
        function x_k = getState(obj, key)
            x_k = obj.x(:, key);
        end

        function x_all = getAllStates(obj)
            x_all = obj.x;   % state_dim x n_vars
        end
    end

    % ==================================================================
    %  PRIVATE METHODS
    % ==================================================================
    methods (Access = private)

        function [J_sp, r_vec] = buildWhitenedSystem(obj)
            % Assemble the full (whitened) Jacobian and residual vector
            % using sparse triplets.

            total_dim = obj.n_vars * obj.state_dim;

            % --- Pre-allocate triplet arrays ---
            % Generous capacity estimate
            cap = obj.n_factors * obj.state_dim * obj.state_dim * 3;
            tri_r = zeros(cap, 1);
            tri_c = zeros(cap, 1);
            tri_v = zeros(cap, 1);
            nnz_cnt = 0;

            res_list = [];     % residual entries accumulator
            row_offset = 0;

            for i = 1:obj.n_factors
                f = obj.factors{i};
                [r_i, J_blocks] = obj.evaluateFactor(f);

                % Whiten
                r_w = f.sqrt_info * r_i;
                res_dim = length(r_w);

                % Append residual
                res_list = [res_list; r_w]; %#ok<AGROW>

                % Append Jacobian blocks
                for b = 1:length(f.keys)
                    key_b = f.keys(b);
                    J_w = f.sqrt_info * J_blocks{b};
                    col_start = (key_b - 1) * obj.state_dim;

                    [nr, nc] = size(J_w);
                    for jr = 1:nr
                        for jc = 1:nc
                            if abs(J_w(jr, jc)) > 1e-15
                                nnz_cnt = nnz_cnt + 1;
                                tri_r(nnz_cnt) = row_offset + jr;
                                tri_c(nnz_cnt) = col_start + jc;
                                tri_v(nnz_cnt) = J_w(jr, jc);
                            end
                        end
                    end
                end

                row_offset = row_offset + res_dim;
            end

            tri_r = tri_r(1:nnz_cnt);
            tri_c = tri_c(1:nnz_cnt);
            tri_v = tri_v(1:nnz_cnt);

            J_sp  = sparse(tri_r, tri_c, tri_v, row_offset, total_dim);
            r_vec = res_list;
        end

        function [r, J_blocks] = evaluateFactor(obj, f)
            % Evaluate a single factor: compute residual and Jacobian blocks.

            switch f.type
                % --------------------------------------------------
                case 'prior'
                    x_k = obj.x(:, f.keys);
                    r = x_k - f.z;
                    J_blocks = {eye(obj.state_dim)};

                % --------------------------------------------------
                case 'dynamics'
                    x_k   = obj.x(:, f.keys(1));
                    x_kp1 = obj.x(:, f.keys(2));
                    dt    = f.dt;

                    % Predicted next state (constant velocity)
                    p_t = x_k(1:3);
                    v_t = x_k(4:6);
                    x_pred = [p_t + v_t * dt; v_t];

                    % Residual
                    r = x_kp1 - x_pred;

                    % Jacobians
                    %   F = df/dx_k = [I, dt*I; 0, I]
                    %   dr/dx_k   = -F
                    %   dr/dx_k+1 = I
                    sd = obj.state_dim;
                    F_k = eye(sd);
                    F_k(1:3, 4:6) = dt * eye(3);

                    J_blocks = { -F_k, eye(sd) };

                % --------------------------------------------------
                case 'camera'
                    x_k = obj.x(:, f.keys);
                    p_t = x_k(1:3);

                    % Target in camera frame
                    M = f.R_b2c * f.R_b2e';
                    p_cam = M * (p_t - f.p_i);

                    % Perspective projection
                    u = p_cam(1);  v = p_cam(2);  w = p_cam(3);
                    w = max(w, 0.01);          % safety clamp
                    h = [u / w; v / w];

                    % Residual
                    r = f.z - h;

                    % Jacobian  dh/dp_t = (dpi/dp_cam) * M
                    dpi_dp = [1/w, 0, -u/w^2;
                              0, 1/w, -v/w^2];
                    dh_dpt = dpi_dp * M;

                    % dr/dx = -[dh/dp_t, 0_{2x3}]
                    J = zeros(2, obj.state_dim);
                    J(:, 1:3) = -dh_dpt;
                    J_blocks = {J};

                % --------------------------------------------------
                case 'radar'
                    x_k = obj.x(:, f.keys);
                    r = f.z - x_k;
                    J_blocks = { -eye(obj.state_dim) };
            end
        end
    end
end

%% ======================== Helper (file-scope) ========================
function S = computeSqrtInfo(cov)
    % Compute the square-root information matrix L^{-1} where cov = L*L'.
    L = chol(cov, 'lower');
    S = L \ eye(size(L));
end
