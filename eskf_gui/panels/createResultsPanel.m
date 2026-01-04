function [components, handles] = createResultsPanel(pix, panelX, panelY, panelWidth, panelHeight, callbacks)
    % createResultsPanel Creates the Results/Stats panel
    
    parent = pix.fig;
    components = [];
    handles = struct();
    bgTitle = [0.85 0.85 0.9];
    
    yPos = 0.96;
    hTitle = uicontrol(parent, 'Style', 'text', 'String', 'SIMULATION STATUS', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    yPos = yPos - 0.05;
    
    % Stats
    uicontrol(parent, 'Style', 'text', 'String', 'Est Error (Position):', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.5, 0.025], 'HorizontalAlignment', 'left');
    handles.txt_err_pos = uicontrol(parent, 'Style', 'text', 'String', '0.00 m', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.35, 0.025], 'HorizontalAlignment', 'right', 'BackgroundColor', 'w');
    yPos = yPos - 0.04;
    
    uicontrol(parent, 'Style', 'text', 'String', 'Est Error (Velocity):', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.5, 0.025], 'HorizontalAlignment', 'left');
    handles.txt_err_vel = uicontrol(parent, 'Style', 'text', 'String', '0.00 m/s', ...
        'Units', 'normalized', 'Position', [0.6, yPos, 0.35, 0.025], 'HorizontalAlignment', 'right', 'BackgroundColor', 'w');
    yPos = yPos - 0.06;
    
    % Plot Selection
    uicontrol(parent, 'Style', 'text', 'String', 'PLOT VIEW', ...
        'Units', 'normalized', 'Position', [0.02, yPos, 0.96, 0.03], ...
        'FontWeight', 'bold', 'FontSize', 9, 'BackgroundColor', bgTitle);
    yPos = yPos - 0.05;
    
    handles.b3D = uicontrol(parent, 'Style', 'pushbutton', 'String', '3D Scenario', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.9, 0.04], 'Tag', 'plot_3d');
    yPos = yPos - 0.05;
    
    handles.bPos = uicontrol(parent, 'Style', 'pushbutton', 'String', 'State Estimation (p,v,pbar)', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.9, 0.04], 'Tag', 'plot_pos');
    yPos = yPos - 0.05;
    
    handles.bAtt = uicontrol(parent, 'Style', 'pushbutton', 'String', 'Attitude Estimation', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.9, 0.04], 'Tag', 'plot_att');
    yPos = yPos - 0.05;
    
    handles.bBias = uicontrol(parent, 'Style', 'pushbutton', 'String', 'IMU Bias Estimation', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.9, 0.04], 'Tag', 'plot_bias');
    yPos = yPos - 0.05;
    
    handles.bCov = uicontrol(parent, 'Style', 'pushbutton', 'String', 'Covariance Evolution', ...
        'Units', 'normalized', 'Position', [0.05, yPos, 0.9, 0.04], 'Tag', 'plot_cov');
    
    % Note: Setup callbacks in main GUI to handle switching axes content
    
end
