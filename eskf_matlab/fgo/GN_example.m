% Simple Gauss-Newton Example: 2D Localization
% Goal: Find the unknown 2D position (x, y) of a target
% based on noisy distance measurements to 3 known beacons.

% 1. Setup the True Scene
rng(42); % Fixed seed for reproducible noise
x_true = [5; 4]; % True target position
beacons = [
    0, 0;  % Beacon 1 (x, y)
    10, 0; % Beacon 2
    0, 10  % Beacon 3
];
num_measurements = size(beacons, 1);

% 2. Generate Noisy Measurements
z = zeros(num_measurements, 1);
for i = 1:num_measurements
    dx = x_true(1) - beacons(i, 1);
    dy = x_true(2) - beacons(i, 2);
    true_dist = sqrt(dx^2 + dy^2);
    z(i) = true_dist + 0.1 * randn(); % Add Gaussian noise (std = 0.1)
end

% 3. Gauss-Newton Optimization
% Initial guess (far from true state to see it work)
x_est = [0.1; 0.1]; 

max_iter = 10;
tol = 1e-4;

disp('Starting Gauss-Newton Optimization...');
for iter = 1:max_iter
    
    % Allocate Residual vector (r) and Jacobian matrix (J)
    r = zeros(num_measurements, 1);
    J = zeros(num_measurements, 2);
    
    % Evaluate Residuals and Jacobian for current x_est
    for i = 1:num_measurements
        dx = x_est(1) - beacons(i, 1);
        dy = x_est(2) - beacons(i, 2);
        
        % Predicted distance h(x)
        dist_pred = sqrt(dx^2 + dy^2);
        
        % Residual: r(x) = Predicted - Measured
        r(i) = dist_pred - z(i);
        
        % Jacobian: partial derivatives of r(x) with respect to x and y
        % J = [dr/dx, dr/dy]
        J(i, 1) = dx / dist_pred;
        J(i, 2) = dy / dist_pred;
    end
    
    % Calculate Cost: 0.5 * sum(r^2)
    cost = 0.5 * (r' * r);
    fprintf('Iter %d: Cost = %10.4f, x = [%6.4f, %6.4f]\n', iter, cost, x_est(1), x_est(2));
    
    % --- GAUSS-NEWTON CORE ---
    H = J' * J;      % Approximate Hessian (Information Matrix)
    b = -J' * r;     % Gradient
    
    % Solve for the update step (delta_x)
    dx_opt = H \ b;
    
    % Update the state estimate
    x_est = x_est + dx_opt;
    % -------------------------
    
    % Check for convergence
    if norm(dx_opt) < tol
        fprintf('Converged after %d iterations!\n\n', iter);
        break;
    end
end

disp('Final Estimate vs True:');
fprintf('Estimated: [%.4f, %.4f]\n', x_est(1), x_est(2));
fprintf('True:      [%.4f, %.4f]\n', x_true(1), x_true(2));
