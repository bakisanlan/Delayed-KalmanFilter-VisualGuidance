function [p_tgt, v_tgt] = propagate_target(p_tgt, v_tgt, t, dt)
% PROPAGATE_TARGET Propagate target dynamics for one time step
%
% Inputs:
%   p_tgt  - Current target position (3x1)
%   v_tgt  - Current target velocity (3x1)
%   t      - Current time (scalar)
%   dt     - Time step (scalar)
%
% Outputs:
%   p_tgt  - Updated target position (3x1)
%   v_tgt  - Updated target velocity (3x1)
%
% Target Motion Models:
%   1. Static target:     a_tgt = [0; 0; 0]
%   2. Constant velocity: a_tgt = [0; 0; 0] with non-zero initial v_tgt
%   3. Maneuvering:       a_tgt = function of time

    % Target acceleration model
    % Currently: hovering with small oscillations
    % Modify this for different target behaviors
    a_tgt = 0.1 * sin(0.3 * t) * 0*[0; 1; 0];
    
    % Propagate velocity and position (trapezoidal integration)
    v_tgt_new = v_tgt + a_tgt * dt;
    p_tgt = p_tgt + 0.5 * (v_tgt + v_tgt_new) * dt;
    v_tgt = v_tgt_new;
end
