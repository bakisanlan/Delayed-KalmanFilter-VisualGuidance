function [components, handles] = createModelPanel(pix, panelX, panelY, panelWidth, panelHeight, callbacks)
    % createModelPanel Creates the Model configuration panel
    
    parent = pix.fig;
    components = [];
    handles = struct();
    
    % Default background color for titles
    bgTitle = [0.85 0.85 0.9];
    
    % --- IMU SECTION ---
    yPos = 0.96;
    hTitle = uicontrol(parent, 'Style', 'text', 'String', 'IMU MODEL (DEAD RECKONING)', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    
    yPos = yPos - 0.04;
    
    % Helper to create slider+edit
    function [hSlider, hEdit] = createSliderGroup(label, y, minV, maxV, initV, tag)
        uicontrol(parent, 'Style', 'text', 'String', label, ...
            'Units', 'normalized', 'Position', [0.05, y, 0.6, 0.025], 'HorizontalAlignment', 'left', 'FontSize', 8);
        
        hEdit = uicontrol(parent, 'Style', 'edit', 'String', num2str(initV, '%g'), ...
            'Units', 'normalized', 'Position', [0.7, y, 0.25, 0.03], 'Tag', [tag '_edit']);
            
        hSlider = uicontrol(parent, 'Style', 'slider', 'Min', minV, 'Max', maxV, 'Value', initV, ...
            'Units', 'normalized', 'Position', [0.05, y-0.03, 0.9, 0.025], 'Tag', [tag '_slider']);
        
        % Link slider and edit (use %g for scientific notation on small values)
        hSlider.Callback = @(src,~) set(hEdit, 'String', num2str(src.Value, '%g'));
        hEdit.Callback = @(src,~) set(hSlider, 'Value', max(minV, min(maxV, str2double(src.String))));
    end

    [handles.sli_acc_n, handles.ed_acc_n] = createSliderGroup('Accel Noise (m/s^2)', yPos, 0, 1.0, 0.1, 'acc_n'); yPos = yPos - 0.07;
    [handles.sli_gyr_n, handles.ed_gyr_n] = createSliderGroup('Gyro Noise (rad/s)', yPos, 0, 0.2, 0.01, 'gyr_n'); yPos = yPos - 0.07;
    [handles.sli_acc_w, handles.ed_acc_w] = createSliderGroup('Accel Bias RW', yPos, 0, 1e-2, 1e-4, 'acc_w'); yPos = yPos - 0.07;
    [handles.sli_gyr_w, handles.ed_gyr_w] = createSliderGroup('Gyro Bias RW', yPos, 0, 1e-3, 1e-5, 'gyr_w'); yPos = yPos - 0.07;
    
    % Frequency
    uicontrol(parent, 'Style', 'text', 'String', 'IMU Frequency (Hz):', ...
         'Units', 'normalized', 'Position', [0.05, yPos, 0.5, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_imu_hz = uicontrol(parent, 'Style', 'edit', 'String', '200', ...
         'Units', 'normalized', 'Position', [0.6, yPos, 0.35, 0.03]);
    yPos = yPos - 0.08;

    % --- SCENARIO SECTION ---
    hTitle2 = uicontrol(parent, 'Style', 'text', 'String', 'SCENARIO SETTINGS', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    yPos = yPos - 0.05;
    
    % Init States
    uicontrol(parent, 'Style', 'text', 'String', 'Int Pos [N E D]:', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_p_int = uicontrol(parent, 'Style', 'edit', 'String', '0 0 -65', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Tgt Pos [N E D]:', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_p_tgt = uicontrol(parent, 'Style', 'edit', 'String', '200 200 -100', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;

    uicontrol(parent, 'Style', 'text', 'String', 'Int v_b (m/s):', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_v_int = uicontrol(parent, 'Style', 'edit', 'String', '5 0 0', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Tgt v_n (m/s):', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_v_tgt = uicontrol(parent, 'Style', 'edit', 'String', '0 0 0', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;
    
    % Initial Attitude
    uicontrol(parent, 'Style', 'text', 'String', 'Int Euler [Y P R] (deg):', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_euler_int = uicontrol(parent, 'Style', 'edit', 'String', '0 0 0', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;
    
    % Max Constraints
    uicontrol(parent, 'Style', 'text', 'String', 'Max Velocity:', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_v_max = uicontrol(parent, 'Style', 'edit', 'String', '30', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Max Body Rate:', 'Units', 'normalized', 'Position', [0.05, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_w_max = uicontrol(parent, 'Style', 'edit', 'String', '2.0', 'Units', 'normalized', 'Position', [0.5, yPos, 0.45, 0.03]);
    
end
