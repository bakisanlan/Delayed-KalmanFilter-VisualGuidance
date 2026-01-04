classdef InterceptorVisualizer < handle
    % INTERCEPTORVISUALIZER specific 3D visualization for interceptor-target scenario
    %
    % Visualizes:
    % - True Interceptor Position (Blue Dot)
    % - True Attitude (Blue Cone)
    % - Estimated Attitude (Red Dashed Cone)
    % - True Target Position (Green Dot)
    % - Estimated Target Position (Magenta Cross) (Derived from p_int - p_r_est)
    
    properties
        fig
        ax
        
        % Graphic Handles
        h_int_path      % animatedline for trajectory
        h_tgt_path      % animatedline for target trajectory
        
        h_int_cone_true % Surface for true attitude
        h_int_cone_est  % Surface for projected/estimated attitude
        
        h_tgt_true      % Scatter for true target
        h_tgt_est       % Scatter for estimated target
        
        h_los_line      % Line connecting interceptor and target
        
        % Cone Geometry
        cone_X
        cone_Y
        cone_Z
    end
    
    methods
        function obj = InterceptorVisualizer(axis_limits)
            % CONSTRUCTOR
            obj.fig = figure('Name', 'Real-Time Interception Visualizer', 'Color', 'w');
            obj.ax = axes('Parent', obj.fig);
            hold(obj.ax, 'on');
            grid(obj.ax, 'on');
            axis(obj.ax, 'equal');
            xlabel(obj.ax, 'North (m)');
            ylabel(obj.ax, 'East (m)');
            zlabel(obj.ax, 'Down (m)');
            set(obj.ax, 'ZDir', 'reverse');
            set(obj.ax, 'YDir', 'reverse'); % NED convention usually has Y East
            view(obj.ax, 3);
            
            if nargin > 0
                axis(obj.ax, axis_limits);
            end
            
            % Initialize Geometry
            [obj.h_int_path] = animatedline('Color', 'b', 'LineWidth', 1.5, 'Parent', obj.ax);
            [obj.h_tgt_path] = animatedline('Color', 'g', 'LineWidth', 1, 'LineStyle', '--', 'Parent', obj.ax);
            
            % Create generic cone oriented along X-axis
            % Height 5m, Radius 1.5m
            [Y, Z, X] = cylinder([0 1.5], 20); 
            X = X * 5; % Scale length
            % Shift so apex is at origin? Cylinder generates from z=0 to z=1.
            % We want apex at origin. 
            % Cylinder(r) -> r(1) at z=0, r(2) at z=1.
            % [0 1.5] -> Apex at X=0, Base at X=5.
            
            obj.cone_X = X;
            obj.cone_Y = Y;
            obj.cone_Z = Z;
            
            % Surfaces
            obj.h_int_cone_true = surf(obj.ax, X, Y, Z, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
            obj.h_int_cone_est  = surf(obj.ax, X, Y, Z, 'FaceColor', 'r', 'EdgeColor', 'r', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.5);
            
            % Points
            obj.h_tgt_true = scatter3(obj.ax, 0, 0, 0, 50, 'g', 'filled');
            obj.h_tgt_est  = scatter3(obj.ax, 0, 0, 0, 50, 'm', 'x', 'LineWidth', 2);
            
            % LOS Line
            obj.h_los_line = plot3(obj.ax, [0 0], [0 0], [0 0], 'k:', 'LineWidth', 0.5);
            
            legend([obj.h_int_cone_true, obj.h_int_cone_est, obj.h_tgt_true, obj.h_tgt_est], ...
                   {'Int True (Body X)', 'Int Est (Body X)', 'Tgt True', 'Tgt Est'});
        end
        
        function update(obj, p_int_true, q_true, q_est, p_tgt_true, p_tgt_est)
            % Update plot
            % Inputs:
            %   p_int_true: [3x1]
            %   q_true:     [4x1] or [1x4]
            %   q_est:      [4x1] or [1x4]
            %   p_tgt_true: [3x1]
            %   p_tgt_est:  [3x1]
            
            % 1. Update positions (Scatter/Lines)
            addpoints(obj.h_int_path, p_int_true(1), p_int_true(2), p_int_true(3));
            addpoints(obj.h_tgt_path, p_tgt_true(1), p_tgt_true(2), p_tgt_true(3));
            
            set(obj.h_tgt_true, 'XData', p_tgt_true(1), 'YData', p_tgt_true(2), 'ZData', p_tgt_true(3));
            set(obj.h_tgt_est, 'XData', p_tgt_est(1), 'YData', p_tgt_est(2), 'ZData', p_tgt_est(3));
            
            set(obj.h_los_line, 'XData', [p_int_true(1) p_tgt_true(1)], ...
                                'YData', [p_int_true(2) p_tgt_true(2)], ...
                                'ZData', [p_int_true(3) p_tgt_true(3)]);
                            
            % 2. Update Cones (Rotation and Translation)
            
            % --- True Cone ---
            R_true = quat2rotm(q_true(:)');
            % Transform generic cone points
            % Flatten arrays
            n_pts = numel(obj.cone_X);
            pts = [obj.cone_X(:)'; obj.cone_Y(:)'; obj.cone_Z(:)'];
            
            % Rotate and Translate
            pts_rot_true = R_true * pts + p_int_true;
            
            % Reshape
            X_new = reshape(pts_rot_true(1,:), size(obj.cone_X));
            Y_new = reshape(pts_rot_true(2,:), size(obj.cone_Y));
            Z_new = reshape(pts_rot_true(3,:), size(obj.cone_Z));
            
            set(obj.h_int_cone_true, 'XData', X_new, 'YData', Y_new, 'ZData', Z_new);
            
            % --- Estimated Cone ---
            R_est = quat2rotm(q_est(:)');
            pts_rot_est = R_est * pts + p_int_true; % Use TRUE position for visualization anchor
            
            X_est = reshape(pts_rot_est(1,:), size(obj.cone_X));
            Y_est = reshape(pts_rot_est(2,:), size(obj.cone_Y));
            Z_est = reshape(pts_rot_est(3,:), size(obj.cone_Z));
            
            set(obj.h_int_cone_est, 'XData', X_est, 'YData', Y_est, 'ZData', Z_est);
            
            % Update visual limits dynamically
            pts_all = [p_int_true, p_tgt_true];
            min_xyz = min(pts_all, [], 2);
            max_xyz = max(pts_all, [], 2);
            
            margin = 5; % meters
            
            % If points are too close, enforce minimum range
            range = max_xyz - min_xyz;
            min_range = 10;
            
            xlim_new = [min_xyz(1) - margin, max_xyz(1) + margin];
            ylim_new = [min_xyz(2) - margin, max_xyz(2) + margin];
            zlim_new = [min_xyz(3) - margin, max_xyz(3) + margin];
            
            axis(obj.ax, [xlim_new, ylim_new, zlim_new]);
        end
    end
end
