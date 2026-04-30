classdef SimpleSlidingWindowFGO < handle
    % SLIDINGWINDOWFGO  Online sliding-window factor graph optimizer.
    %
    % Maintains a window of at most W variable nodes.  At each time step a
    % new node is appended, factors are attached, and Gauss-Newton is run
    % over the window.  When the window reaches capacity the oldest node is
    % marginalised via the Schur complement, producing a dense
    % "marginalisation prior" on the remaining nodes that carries
    % information from the evicted history.
    %
    %  State per node : state_dim-dimensional column vector.
    %
    % ------------------------------------------------------------------
    % Typical usage
    % ------------------------------------------------------------------
    %   sw = SlidingWindowFGO(W, state_dim);
    %   sw.initFirstNode(1, x0, P0);
    %
    %   for k = 2 : N
    %       if sw.n_active == sw.W
    %           sw.marginalizeOldestNode();   % Schur-complement eviction
    %       end
    %       sw.addNewNode(k, x_init_k);
    %       sw.addDynamicsFactor(k-1, k, dt, Q);
    %       if cam_valid(capture_k) && sw.isInWindow(capture_k)
    %           sw.addCameraFactor(capture_k, z, R, R_b2c, R_b2e, p_i);
    %       end
    %       if radar at k
    %           sw.addRadarFactor(k, z, R);
    %       end
    %       sw.optimizeWindow(max_iter, tol);
    %       x_k = sw.getNewestState();
    %   end
    % ------------------------------------------------------------------

    properties
        W              % Maximum window size  [scalar int]
        state_dim      % Dimension per node   [scalar int]
        abs_indices    % Absolute node IDs in window  [1 x W] (first n_active cols valid)
        n_active       % Number of nodes currently in window  [scalar int]
        x              % Node estimates  [state_dim x W]  (cols 1..n_active used)
        factors        % Cell array of active factor structs
        n_factors      % Number of stored factors
        has_marg_prior % True once a marginalisation prior has been computed
        marg_prior     % Struct: marginalization prior (dense Gaussian on b-nodes)
    end

    methods

        % ==============================================================
        %  CONSTRUCTOR
        % ==============================================================
        function obj = SimpleSlidingWindowFGO(W, state_dim)
            obj.W              = W;
            obj.state_dim      = state_dim;
            obj.abs_indices    = zeros(1, W);
            obj.n_active       = 0;
            obj.x              = zeros(state_dim, W);
            obj.factors        = {};
            obj.n_factors      = 0;
            obj.has_marg_prior = false;
            obj.marg_prior     = [];
        end

        % ==============================================================
        %  INITIALISATION
        % ==============================================================
        function initFirstNode(obj, abs_k, x_init, P_prior)
            % Seed the window with the first variable node + Gaussian prior.
            assert(obj.n_active == 0, ...
                'initFirstNode must be called on an empty window.');
            obj.n_active       = 1;
            obj.abs_indices(1) = abs_k;
            obj.x(:, 1)        = x_init(:);

            f.type      = 'prior';
            f.keys      = abs_k;
            f.z         = x_init(:);
            f.sqrt_info = swfgo_sqrtinfo(P_prior);
            obj.n_factors      = 1;
            obj.factors        = {f};
        end

        % ==============================================================
        %  NODE MANAGEMENT
        % ==============================================================
        function addNewNode(obj, abs_k, x_init)
            % Append node abs_k to the right of the window.
            % Precondition: n_active < W  (call marginalizeOldestNode first).
            assert(obj.n_active < obj.W, ...
                'Window is full — call marginalizeOldestNode() before addNewNode().');
            obj.n_active = obj.n_active + 1;
            obj.abs_indices(obj.n_active) = abs_k;
            obj.x(:, obj.n_active)        = x_init(:);
        end

        function tf = isInWindow(obj, abs_k)
            % Return true if absolute node abs_k is currently in the window.
            tf = any(obj.abs_indices(1:obj.n_active) == abs_k);
        end

        % ==============================================================
        %  FACTOR ADDITION  (absolute node indices)
        % ==============================================================
        function addDynamicsFactor(obj, abs_k, abs_kp1, dt, cov)
            % Constant-velocity between factor:  x_{k+1} = F * x_k + w
            f.type      = 'dynamics';
            f.keys      = [abs_k, abs_kp1];
            f.dt        = dt;
            f.sqrt_info = swfgo_sqrtinfo(cov);
            obj.n_factors             = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        function addCameraFactor(obj, abs_k, z_meas, cov, R_b2c, R_b2e, p_i)
            % Perspective-projection measurement factor.
            f.type      = 'camera';
            f.keys      = abs_k;
            f.z         = z_meas(:);
            f.sqrt_info = swfgo_sqrtinfo(cov);
            f.R_b2c     = R_b2c;
            f.R_b2e     = R_b2e;
            f.p_i       = p_i(:);
            obj.n_factors             = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        function addRadarFactor(obj, abs_k, z_radar, cov)
            % Direct position + velocity measurement factor.
            f.type      = 'radar';
            f.keys      = abs_k;
            f.z         = z_radar(:);
            f.sqrt_info = swfgo_sqrtinfo(cov);
            obj.n_factors             = obj.n_factors + 1;
            obj.factors{obj.n_factors} = f;
        end

        % ==============================================================
        %  GAUSS-NEWTON OPTIMISATION (window-local)
        % ==============================================================
        function cost_hist = optimizeWindow(obj, max_iter, tol)
            % Run Gauss-Newton over the current window.
            %   min  0.5 * ||r_w(x)||^2
            % Returns per-iteration cost history (trimmed at convergence).
            sd         = obj.state_dim;
            cost_hist  = zeros(max_iter, 1);

            for iter = 1:max_iter
                [J, r] = obj.buildWhitenedSystem();

                cost             = 0.5 * (r' * r);
                cost_hist(iter)  = cost;

                H  = J' * J;
                b  = -J' * r;
                H  = H + 1e-8 * speye(size(H));   % LM-style damping
                dx = H \ b;

                for s = 1:obj.n_active
                    idx        = (s-1)*sd + (1:sd);
                    obj.x(:,s) = obj.x(:,s) + dx(idx);
                end

                if norm(dx) < tol
                    cost_hist = cost_hist(1:iter);
                    return;
                end
            end
            cost_hist = cost_hist(1:max_iter);
        end

        % ==============================================================
        %  MARGINALISATION  (Schur complement of oldest node)
        % ==============================================================
        function marginalizeOldestNode(obj)
            % Evict window slot 1 (oldest absolute node) via Schur complement.
            %
            % Math:
            %   Partition full Hessian:  H = [H_aa, H_ab; H_ba, H_bb]
            %                            b = [b_a;  b_b ]
            %   where  a = slot 1, b = slots 2..n_active.
            %
            %   Schur complement onto b:
            %     H_marg = H_bb  -  H_ba * H_aa^{-1} * H_ab
            %     b_marg = b_b   -  H_ba * H_aa^{-1} * b_a
            %
            %   Future cost: 0.5 * (x_b - mu)' H_marg (x_b - mu)
            %   where  mu = x_b_lin + H_marg^{-1} * b_marg
            %
            assert(obj.n_active >= 2, ...
                'Need at least 2 nodes in the window to marginalize.');

            sd = obj.state_dim;

            % Build full whitened Hessian and gradient
            [J, r] = obj.buildWhitenedSystem();
            H      = full(J' * J);
            b      = full(-J' * r);

            % Partition indices
            n_a   = sd;                          % evicted node  (slot 1)
            n_b   = (obj.n_active - 1) * sd;     % remaining nodes (slots 2..n_active)
            ia    = 1:n_a;
            ib    = (n_a + 1):(n_a + n_b);

            H_aa  = H(ia, ia) + 1e-9 * eye(n_a);  % regularise for safety
            H_ab  = H(ia, ib);
            H_ba  = H(ib, ia);
            H_bb  = H(ib, ib);
            b_a   = b(ia);
            b_b   = b(ib);

            % Solve  H_aa \ [H_ab, b_a]  in one pass (efficiency)
            rhs       = [H_ab, b_a];
            sol       = H_aa \ rhs;
            H_marg    = H_bb - H_ba * sol(:, 1:n_b);
            b_marg    = b_b  - H_ba * sol(:, end);

            % Symmetrize & regularize Schur complement
            H_marg = 0.5 * (H_marg + H_marg') + 1e-9 * eye(n_b);

            % Linearization point for remaining nodes (stacked column vector)
            x_b_lin  = obj.x(:, 2:obj.n_active);
            x_b_vec  = x_b_lin(:);

            % mu  = x_b_lin + H_marg^{-1} * b_marg
            L_marg    = chol(H_marg, 'lower');       % H_marg = L * L'
            dx_b_opt  = L_marg' \ (L_marg \ b_marg); % H_marg^{-1} * b_marg
            mu_marg   = x_b_vec + dx_b_opt;

            % sqrt_info = L'  so that  sqrt_info' * sqrt_info = L * L' = H_marg
            sqrt_info_marg = L_marg';

            % Build marginalization prior struct
            mp.keys      = obj.abs_indices(2:obj.n_active);  % absolute IDs
            mp.mu        = mu_marg;
            mp.sqrt_info = sqrt_info_marg;
            obj.marg_prior     = mp;
            obj.has_marg_prior = true;

            % Evict oldest node: shift window arrays left
            obj.abs_indices(1:obj.n_active-1) = obj.abs_indices(2:obj.n_active);
            obj.x(:, 1:obj.n_active-1)        = obj.x(:, 2:obj.n_active);
            obj.n_active = obj.n_active - 1;

            % Prune factors whose keys are no longer in the window
            obj.pruneOutdatedFactors();
        end

        % ==============================================================
        %  STATE RETRIEVAL
        % ==============================================================
        function x_k = getNewestState(obj)
            x_k = obj.x(:, obj.n_active);
        end

        function x_all = getAllWindowStates(obj)
            % Returns state_dim x n_active matrix (column order = oldest..newest)
            x_all = obj.x(:, 1:obj.n_active);
        end

        function abs_idx = getWindowAbsIndices(obj)
            abs_idx = obj.abs_indices(1:obj.n_active);
        end

        function P_k = getNewestMarginal(obj)
            % Marginal covariance of the newest (right-most) node.
            % Uses sparse back-substitution — does not form the full inverse.
            [J, ~] = obj.buildWhitenedSystem();
            sd     = obj.state_dim;
            H      = J' * J + 1e-10 * speye(obj.n_active * sd);
            idx    = (obj.n_active - 1)*sd + (1:sd);
            n      = size(H, 1);
            E      = sparse(idx, 1:sd, ones(sd, 1), n, sd);
            C      = H \ E;
            P_k    = full(C(idx, :));
            P_k    = 0.5 * (P_k + P_k');
        end

    end % public methods

    % ==================================================================
    %  PRIVATE METHODS
    % ==================================================================
    methods (Access = private)

        % --------------------------------------------------------------
        function [J_sp, r_vec] = buildWhitenedSystem(obj)
            % Assemble the whitened Jacobian (sparse) and residual vector
            % for all active factors and the marginalisation prior.
            % (SIMPLIFIED VERSION FOR READABILITY)

            sd        = obj.state_dim;
            n_tot_dim = obj.n_active * sd;

            J_dense = [];
            r_vec   = [];

            % --- Factor contributions ---
            for i = 1:obj.n_factors
                f = obj.factors{i};

                % Resolve all keys to window slots
                n_keys = length(f.keys);
                slots  = zeros(1, n_keys);
                ok     = true;
                for bi = 1:n_keys
                    s = find(obj.abs_indices(1:obj.n_active) == f.keys(bi), 1);
                    if isempty(s); ok = false; break; end
                    slots(bi) = s;
                end
                if ~ok, continue; end   % skip if any key has been evicted

                [r_i, J_blks] = obj.evaluateFactor(f, slots);
                r_w     = f.sqrt_info * r_i;
                r_vec   = [r_vec; r_w]; %#ok<AGROW>
                
                % Build the dense row block for this factor
                res_dim = length(r_w);
                J_row = zeros(res_dim, n_tot_dim);

                for bi = 1:n_keys
                    J_w     = f.sqrt_info * J_blks{bi};
                    col_idx = (slots(bi) - 1) * sd + 1;
                    J_row(:, col_idx : col_idx + sd - 1) = J_w;
                end
                J_dense = [J_dense; J_row]; %#ok<AGROW>
            end

            % --- Marginalisation prior contribution ---
            if obj.has_marg_prior
                mp   = obj.marg_prior;
                n_mp = length(mp.keys);
                n_b  = n_mp * sd;

                % Resolve mp keys to current window slots
                slots_mp = zeros(1, n_mp);
                ok       = true;
                for bi = 1:n_mp
                    s = find(obj.abs_indices(1:obj.n_active) == mp.keys(bi), 1);
                    if isempty(s); ok = false; break; end
                    slots_mp(bi) = s;
                end

                if ok
                    % Stack current x_b in mp.keys order
                    x_b_cur = zeros(n_b, 1);
                    for bi = 1:n_mp
                        x_b_cur((bi-1)*sd + (1:sd)) = obj.x(:, slots_mp(bi));
                    end

                    % Whitened residual
                    r_mp   = x_b_cur - mp.mu;
                    r_w_mp = mp.sqrt_info * r_mp;
                    r_vec  = [r_vec; r_w_mp]; %#ok<AGROW>

                    % Dense whitened Jacobian: J_w = sqrt_info (identity Jacobian)
                    J_full = mp.sqrt_info;   % n_b x n_b
                    J_row  = zeros(n_b, n_tot_dim);

                    for bi = 1:n_mp
                        col_in_full = (bi-1)*sd + 1;
                        J_blk = J_full(:, col_in_full : col_in_full + sd - 1);
                        
                        col_in_J_dense = (slots_mp(bi) - 1) * sd + 1;
                        J_row(:, col_in_J_dense : col_in_J_dense + sd - 1) = J_blk;
                    end
                    J_dense = [J_dense; J_row]; %#ok<AGROW>
                end
            end

            % Assemble final sparse matrix
            J_sp = sparse(J_dense);
        end

        % --------------------------------------------------------------
        function [r, J_blocks] = evaluateFactor(obj, f, slots)
            % Evaluate residual and Jacobian blocks for a single factor.
            %
            % slots(i) = window slot of f.keys(i)   (1-indexed).

            sd = obj.state_dim;

            switch f.type

                % --- Prior ---
                case 'prior'
                    x_k = obj.x(:, slots(1));
                    r   = x_k - f.z;
                    J_blocks = {eye(sd)};

                % --- Constant-velocity dynamics ---
                case 'dynamics'
                    x_k   = obj.x(:, slots(1));
                    x_kp1 = obj.x(:, slots(2));
                    dt    = f.dt;
                    p_t   = x_k(1:3);
                    v_t   = x_k(4:6);
                    x_pred = [p_t + v_t*dt; v_t];
                    r      = x_kp1 - x_pred;
                    F_k    = eye(sd);
                    F_k(1:3, 4:6) = dt * eye(3);
                    J_blocks = {-F_k, eye(sd)};

                % --- Perspective camera ---
                case 'camera'
                    x_k   = obj.x(:, slots(1));
                    p_t   = x_k(1:3);
                    M     = f.R_b2c * f.R_b2e';
                    p_cam = M * (p_t - f.p_i);
                    u = p_cam(1);  v = p_cam(2);  w = max(p_cam(3), 0.01);
                    h = [u/w; v/w];
                    r = f.z - h;
                    dpi_dp = [1/w,   0,  -u/w^2;
                              0,   1/w,  -v/w^2];
                    J = zeros(2, sd);
                    J(:, 1:3) = -dpi_dp * M;
                    J_blocks  = {J};

                % --- Radar (direct pos+vel) ---
                case 'radar'
                    x_k = obj.x(:, slots(1));
                    r   = f.z - x_k;
                    J_blocks = {-eye(sd)};

                otherwise
                    error('SlidingWindowFGO: unknown factor type ''%s''.', f.type);
            end
        end

        % --------------------------------------------------------------
        function pruneOutdatedFactors(obj)
            % Remove factors that have ANY key no longer in the current window.
            % (Their information has been absorbed into the marginalisation prior.)
            win  = obj.abs_indices(1:obj.n_active);
            keep = true(1, obj.n_factors);
            for i = 1:obj.n_factors
                for key = obj.factors{i}.keys
                    if ~any(win == key)
                        keep(i) = false;
                        break;
                    end
                end
            end
            obj.factors   = obj.factors(keep);
            obj.n_factors = sum(keep);
        end

    end % private methods
end

% ======================================================================
%  File-scope helper
% ======================================================================
function S = swfgo_sqrtinfo(cov)
    % Square-root information matrix:  S = L^{-1}  where  cov = L * L'.
    % Satisfies  S' * S = cov^{-1}  (the information matrix).
    L = chol(cov, 'lower');
    S = L \ eye(size(L));
end
