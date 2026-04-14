function pbar = compute_image_features(p_r, q, R_b2c)
% COMPUTE_IMAGE_FEATURES Compute normalized image coordinates from relative position
%
% Inputs:
%   p_r   - Relative position in earth frame (3x1)
%   q     - Quaternion [q0; q1; q2; q3] (4x1)
%   R_b2c - Rotation from body to camera frame (3x3)
%
% Outputs:
%   pbar  - Normalized image coordinates [pbar_x; pbar_y] (2x1)
%   p_zc  - Depth in camera frame (scalar)

    % Get rotation matrix from body to earth
    R_b2e = quat2rotm(q');
    
    % Transform relative position to camera frame
    % Target position in camera = R_b2c * R_b2e' * (-p_r)
    % Note: -p_r because p_r = p_interceptor - p_target
    p_r_cam = R_b2c * R_b2e' * (-p_r);
    
    % Compute normalized coordinates
    if p_r_cam(3) > 2  % Target must be in front of camera
        pbar = [p_r_cam(1) / p_r_cam(3); 
                p_r_cam(2) / p_r_cam(3)];
    else
        pbar = [0; 0];  % Invalid measurement
        disp('depth is approaching 0, watch out!')
    end
end
