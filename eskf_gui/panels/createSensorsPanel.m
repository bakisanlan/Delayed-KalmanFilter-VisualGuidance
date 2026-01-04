function [components, handles] = createSensorsPanel(pix, panelX, panelY, panelWidth, panelHeight, callbacks)
    % createSensorsPanel Creates the Sensors configuration panel
    
    parent = pix.fig;
    components = [];
    handles = struct();
    bgTitle = [0.85 0.85 0.9];
    
    yPos = 0.96;
    hTitle = uicontrol(parent, 'Style', 'text', 'String', 'SENSOR CONFIGURATION', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    yPos = yPos - 0.05;
    
    % --- CAMERA ---
    uicontrol(parent, 'Style', 'text', 'String', 'Camera', 'FontWeight', 'bold', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.3, 0.025], 'HorizontalAlignment', 'left');
    handles.chk_cam = uicontrol(parent, 'Style', 'checkbox', 'String', 'Enabled', 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.5, yPos, 0.4, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Update Rate (Hz):', ...
        'Units', 'normalized', 'Position', [0.1, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_cam_hz = uicontrol(parent, 'Style', 'edit', 'String', '30', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.3, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Image Noise (px/norm):', ...
        'Units', 'normalized', 'Position', [0.1, yPos, 0.5, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_cam_noise = uicontrol(parent, 'Style', 'edit', 'String', '0.005', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.3, 0.03]);
    yPos = yPos - 0.05;

    uicontrol(parent, 'Style', 'text', 'String', 'Latency (ms):', ...
        'Units', 'normalized', 'Position', [0.1, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_cam_delay = uicontrol(parent, 'Style', 'edit', 'String', '0', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.3, 0.03]);
    yPos = yPos - 0.08;
    
    % --- RADAR ---
    uicontrol(parent, 'Style', 'text', 'String', 'Radar', 'FontWeight', 'bold', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.3, 0.025], 'HorizontalAlignment', 'left');
    handles.chk_radar = uicontrol(parent, 'Style', 'checkbox', 'String', 'Enabled', 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.5, yPos, 0.4, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Update Rate (Hz):', ...
        'Units', 'normalized', 'Position', [0.1, yPos, 0.4, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_radar_hz = uicontrol(parent, 'Style', 'edit', 'String', '0.5', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.3, 0.03]);
    yPos = yPos - 0.05;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Pos Noise (m):', ...
        'Units', 'normalized', 'Position', [0.1, yPos, 0.5, 0.025], 'HorizontalAlignment', 'left');
    handles.ed_radar_noise = uicontrol(parent, 'Style', 'edit', 'String', '1.0', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.3, 0.03]);
    yPos = yPos - 0.05;
end
