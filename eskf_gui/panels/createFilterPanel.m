function [components, handles] = createFilterPanel(pix, panelX, panelY, panelWidth, panelHeight, callbacks)
    % createFilterPanel Creates the Filter configuration panel
    
    parent = pix.fig;
    components = [];
    handles = struct();
    bgTitle = [0.85 0.85 0.9];
    
    yPos = 0.96;
    hTitle = uicontrol(parent, 'Style', 'text', 'String', 'FILTER SETTINGS', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    yPos = yPos - 0.05;
    
    % Filter Type
    uicontrol(parent, 'Style', 'text', 'String', 'Algorithm:', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.3, 0.025], 'HorizontalAlignment', 'left');
    handles.pop_filter = uicontrol(parent, 'Style', 'popupmenu', 'String', {'ESKF', 'UKF (Future)'}, ...
        'Units', 'normalized', 'Position', [0.4, yPos, 0.5, 0.03]);
    yPos = yPos - 0.06;
    
    % Presets
    uicontrol(parent, 'Style', 'text', 'String', 'Uncertainty Presets:', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.6, 0.025], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
    yPos = yPos - 0.04;
    
    bLow = uicontrol(parent, 'Style', 'pushbutton', 'String', 'Low', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.25, 0.04], 'Callback', @(~,~) setPresets('low'));
    bMed = uicontrol(parent, 'Style', 'pushbutton', 'String', 'Medium', ...
        'Units', 'normalized', 'Position', [0.35, yPos, 0.25, 0.04], 'Callback', @(~,~) setPresets('med'));
    bHigh = uicontrol(parent, 'Style', 'pushbutton', 'String', 'High', ...
        'Units', 'normalized', 'Position', [0.65, yPos, 0.25, 0.04], 'Callback', @(~,~) setPresets('high'));
    yPos = yPos - 0.06;
    
    % Fields
    uicontrol(parent, 'Style', 'text', 'String', 'Initial 1-Sigma values:', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.8, 0.025], 'HorizontalAlignment', 'left');
    yPos = yPos - 0.03;

    function [hEdit] = createField(label, y, initV, tag)
        uicontrol(parent, 'Style', 'text', 'String', label, ...
            'Units', 'normalized', 'Position', [0.1, y, 0.5, 0.025], 'HorizontalAlignment', 'left');
        hEdit = uicontrol(parent, 'Style', 'edit', 'String', num2str(initV), ...
            'Units', 'normalized', 'Position', [0.6, y, 0.3, 0.03], 'Tag', tag);
    end

    handles.ed_sig_pos = createField('Position (m)', yPos, 3, 'sig_pos'); yPos = yPos - 0.05;
    handles.ed_sig_vel = createField('Velocity (m/s)', yPos, 0.5, 'sig_vel'); yPos = yPos - 0.05;
    handles.ed_sig_att = createField('Attitude (rad)', yPos, 0.05, 'sig_att'); yPos = yPos - 0.05;
    handles.ed_sig_pbar = createField('Image (norm)', yPos, 0.1, 'sig_pbar'); yPos = yPos - 0.05;
    handles.ed_sig_bg = createField('Gyro Bias (rad/s)', yPos, 0.005, 'sig_bg'); yPos = yPos - 0.05;
    handles.ed_sig_ba = createField('Accel Bias (m/s^2)', yPos, 0.05, 'sig_ba'); yPos = yPos - 0.05;
    
    function setPresets(level)
        switch level
            case 'low'
                vals = {0.5, 0.1, 0.01, 0.05, 0.001, 0.01};
            case 'med'
                vals = {3, 0.5, 0.05, 0.1, 0.005, 0.05};
            case 'high'
                vals = {10, 2.0, 0.2, 0.5, 0.02, 0.2};
        end
        handles.ed_sig_pos.String = num2str(vals{1});
        handles.ed_sig_vel.String = num2str(vals{2});
        handles.ed_sig_att.String = num2str(vals{3});
        handles.ed_sig_pbar.String = num2str(vals{4});
        handles.ed_sig_bg.String = num2str(vals{5});
        handles.ed_sig_ba.String = num2str(vals{6});
    end
end
