function [omega_meas, a_meas] = generate_imu_measurements(omega_true, a_body_true, ...
                                                          b_gyr, b_acc, sigma_gyr, sigma_acc)
% GENERATE_IMU_MEASUREMENTS Generate noisy IMU measurements from true values
%
% Inputs:
%   omega_true   - True angular velocity (3x1) [rad/s]
%   a_body_true  - True body-frame acceleration (3x1) [m/s^2]
%   b_gyr        - Gyroscope bias (3x1) [rad/s]
%   b_acc        - Accelerometer bias (3x1) [m/s^2]
%   sigma_gyr    - Gyroscope noise standard deviation (scalar) [rad/s]
%   sigma_acc    - Accelerometer noise standard deviation (scalar) [m/s^2]
%
% Outputs:
%   omega_meas   - Measured angular velocity (3x1) [rad/s]
%   a_meas       - Measured body-frame acceleration (3x1) [m/s^2]
%
% IMU Measurement Model:
%   omega_meas = omega_true + b_gyr + noise_gyr
%   a_meas     = a_body_true + b_acc + noise_acc

    % Add bias and Gaussian noise
    omega_meas = omega_true + b_gyr + sigma_gyr * randn(3, 1);
    a_meas = a_body_true + b_acc + sigma_acc * randn(3, 1);
end
