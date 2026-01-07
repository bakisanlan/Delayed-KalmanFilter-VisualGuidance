function ESKF_GUI()
    % ESKF_GUI Main Entry Point for ESKF Visual Servoing Project
    
    %% 1. Setup Environment
    % Add paths relative to this file
    guiPath = fileparts(mfilename('fullpath'));
    basePath = fileparts(guiPath); % Parent directory (root of repo)
    
    addpath(guiPath);
    addpath(fullfile(guiPath, 'panels'));
    % Add parent directory for core classes (ESKF, IMUModel, etc.)
    addpath(basePath); 
    % addpath(fullfile(basePath, 'functions')); 
    
    %% 2. Simulation State & Object
    sim = ESKFSimulation(); % Initialize Simulation Engine
    
    appState = struct();
    appState.isRunning = false;
    appState.isPaused = false;
    appState.isLive = true;
    appState.pace = 1.0; % Simulation speed factor
    appState.timerPeriod = 0.05; % Base GUI update rate (20Hz)
    appState.currentView = '3D'; % '3D', 'POS', 'VEL', 'ATT', 'BIAS'
    
    % Timer for Live Simulation
    simTimer = timer('ExecutionMode', 'fixedRate', ...
                     'Period', appState.timerPeriod, ...
                     'TimerFcn', @onTimerTick);
                     
    %% 3. Main Figure
    fig = figure('Name', 'ESKF Visual Servoing Workbench', ...
                 'NumberTitle', 'off', ...
                 'Units', 'normalized', ...
                 'Position', [0.1, 0.1, 0.8, 0.8], ...
                 'WindowState', 'maximized', ...
                 'CloseRequestFcn', @onClose);
    
    % Store data in figure
    uidata = struct();
    uidata.sim = sim;
    
    %% 4. Layout Configuration
    % Left side: 70% for Plots
    % Right side: 30% for Panels
    
    % Main Plot Area Container
    axPositions = [0.02, 0.05, 0.68, 0.9];
    hPlotPanel = uipanel('Parent', fig, 'Units', 'normalized', 'Position', axPositions, ...
                         'BorderType', 'none', 'BackgroundColor', 'w');
    
    % Store current plot handles
    plotHandles = struct('axes', [], 'lines', [], 'type', '');
    
    % Right Panel Container (Panel Area)
    panelX = 0.72;
    panelY = 0.15; % Leave space for bottom buttons
    panelW = 0.26;
    panelH = 0.80;
    
    % Panel Switcher Buttons (Top of Right Side)
    btnH = 0.05;
    btnW = panelW / 4;
    btnY = panelY + panelH + 0.005;
    
    colMap = [0.94 0.94 0.94]; % Default bg
    
    bModel = uicontrol(fig, 'Style', 'pushbutton', 'String', 'MODEL', ...
        'Units', 'normalized', 'Position', [panelX, btnY, btnW, btnH], ...
        'Callback', {@switchPanel, 'MODEL'});
        
    bSensors = uicontrol(fig, 'Style', 'pushbutton', 'String', 'SENSORS', ...
        'Units', 'normalized', 'Position', [panelX+btnW, btnY, btnW, btnH], ...
        'Callback', {@switchPanel, 'SENSORS'});
        
    bFilter = uicontrol(fig, 'Style', 'pushbutton', 'String', 'FILTER', ...
        'Units', 'normalized', 'Position', [panelX+2*btnW, btnY, btnW, btnH], ...
        'Callback', {@switchPanel, 'FILTER'});
        
    bResults = uicontrol(fig, 'Style', 'pushbutton', 'String', 'RESULTS', ...
        'Units', 'normalized', 'Position', [panelX+3*btnW, btnY, btnW, btnH], ...
        'Callback', {@switchPanel, 'RESULTS'});
        
    % Shared handles structure for panels
    panelHandles = struct();
    
    % Pix structure for passing layout info (using pixel conversion if needed, but normalized is easier)
    % For sub-panels, we'll pass the normalized coordinates
    
    %% 5. Initialize Panels
    % We create a container panel for each to easily toggle visibility
    
    % MODEL PANEL
    pModel = uipanel(fig, 'Units', 'normalized', 'Position', [panelX, panelY, panelW, panelH], ...
                     'Title', '', 'BorderType', 'none', 'Visible', 'on');
    [~, panelHandles.model] = createModelPanel(struct('fig', pModel), 0, 0, 1, 1, []);
    
    % SENSORS PANEL
    pSensors = uipanel(fig, 'Units', 'normalized', 'Position', [panelX, panelY, panelW, panelH], ...
                     'Title', '', 'BorderType', 'none', 'Visible', 'off');
    [~, panelHandles.sensors] = createSensorsPanel(struct('fig', pSensors), 0, 0, 1, 1, []);
    
    % FILTER PANEL
    pFilter = uipanel(fig, 'Units', 'normalized', 'Position', [panelX, panelY, panelW, panelH], ...
                     'Title', '', 'BorderType', 'none', 'Visible', 'off');
    [~, panelHandles.filter] = createFilterPanel(struct('fig', pFilter), 0, 0, 1, 1, []);
    
    % RESULTS PANEL
    pResults = uipanel(fig, 'Units', 'normalized', 'Position', [panelX, panelY, panelW, panelH], ...
                     'Title', '', 'BorderType', 'none', 'Visible', 'off');
    [~, panelHandles.results] = createResultsPanel(struct('fig', pResults), 0, 0, 1, 1, []);
    
    % Attach Plot Switching Callbacks
    panelHandles.results.b3D.Callback = {@changeView, '3D'};
    panelHandles.results.bPos.Callback = {@changeView, 'STATE'};
    panelHandles.results.bAtt.Callback = {@changeView, 'ATT'};
    panelHandles.results.bBias.Callback = {@changeView, 'BIAS'};
    panelHandles.results.bCov.Callback = {@changeView, 'COV'};
    
    %% 6. Control Panel (Bottom Right)
    ctrlY = 0.02;
    ctrlH = 0.10;
    
    % Sim Controls
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'RUN', ...
        'Units', 'normalized', 'Position', [panelX, ctrlY+0.06, 0.06, 0.05], ...
        'BackgroundColor', [0.2 0.8 0.2], 'Callback', @onRun);
        
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'STOP', ...
        'Units', 'normalized', 'Position', [panelX+0.07, ctrlY+0.06, 0.06, 0.05], ...
        'BackgroundColor', [0.8 0.2 0.2], 'Callback', @onStop);
        
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'CONTINUE', ...
        'Units', 'normalized', 'Position', [panelX+0.14, ctrlY+0.06, 0.06, 0.05], ...
        'Callback', @onContinue);
        
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'RESET', ...
        'Units', 'normalized', 'Position', [panelX+0.21, ctrlY+0.06, 0.05, 0.05], ...
        'Callback', @onReset);
        
    % Lower Row: Live Toggle, Pace, Save
    uicontrol(fig, 'Style', 'checkbox', 'String', 'Live Sim', ...
        'Units', 'normalized', 'Position', [panelX, ctrlY+0.01, 0.05, 0.04], ...
        'Value', 1, 'Callback', @onLiveToggle);
        
    uicontrol(fig, 'Style', 'text', 'String', 'Pace:', ...
        'Units', 'normalized', 'Position', [panelX+0.05, ctrlY, 0.03, 0.04], ...
        'HorizontalAlignment', 'right');
        
    hPace = uicontrol(fig, 'Style', 'edit', 'String', '1.0', ...
        'Units', 'normalized', 'Position', [panelX+0.09, ctrlY+0.01, 0.04, 0.04], ...
        'Callback', @onPaceChange);
        
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'SAVE RESULTS', ...
        'Units', 'normalized', 'Position', [panelX+0.15, ctrlY+0.01, 0.11, 0.04], ...
        'Callback', @onSave);
        
    %% 7. Callback Functions
    
    function switchPanel(~, ~, name)
        pModel.Visible = 'off';
        pSensors.Visible = 'off';
        pFilter.Visible = 'off';
        pResults.Visible = 'off';
        
        switch name
            case 'MODEL', pModel.Visible = 'on';
            case 'SENSORS', pSensors.Visible = 'on';
            case 'FILTER', pFilter.Visible = 'on';
            case 'RESULTS', pResults.Visible = 'on';
        end
    end

    function onRun(~,~)
        if appState.isRunning, return; end
        
        % Read Config from Panels
        try
            userConfig = readConfigFromGUI();
        catch ME
            errordlg(['Invalid Configuration: ' ME.message], 'Config Error');
            return;
        end
        
        appState.isRunning = true;
        appState.isPaused = false;
        
        sim.initialize(userConfig); % Reload config
        
        if appState.isLive
            start(simTimer);
        else
            % Batch run
            hWait = waitbar(0, 'Running Simulation...');
            try
                while ~sim.step()
                   if mod(sim.k, 100) == 0 && isvalid(hWait)
                       waitbar(sim.t / sim.config.t_total, hWait);
                   end
                end
            catch ME
                if isvalid(hWait), close(hWait); end
                appState.isRunning = false;
                errordlg(ME.message, 'Simulation Error');
                return;
            end
            if isvalid(hWait), close(hWait); end
            appState.isRunning = false;
            plotResults();
        end
    end
    
    function onStop(~,~)
        % Pause the simulation (don't fully stop it)
        if appState.isRunning && ~appState.isPaused
            stop(simTimer);
            appState.isPaused = true;
            % isRunning stays true so CONTINUE can resume
        end
    end
    
    function onContinue(~,~)
        if appState.isRunning && appState.isPaused
             appState.isPaused = false;
             if appState.isLive
                 start(simTimer);
             end
        end
    end
    
    function onReset(~,~)
        stop(simTimer);
        appState.isRunning = false;
        sim.initialize();
        % Reset viz by forcing view reload
        v = appState.currentView;
        appState.currentView = ''; 
        changeView([], [], v);
    end
    
    function changeView(~, ~, viewName)
        if strcmp(appState.currentView, viewName), return; end
        appState.currentView = viewName;
        
        % Clear existing
        delete(hPlotPanel.Children);
        plotHandles = struct('axes', [], 'lines', [], 'type', viewName);
        
        switch viewName
            case '3D'
                setup3DView();
            case 'STATE'
                setupStateView();
            case 'ATT'
                setupAttView();
            case 'BIAS'
                setupBiasView();
            case 'COV'
                setupCovView();
        end
        
        updateVisualization();
    end

    function setup3DView()
        ax = axes('Parent', hPlotPanel, 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);
        grid(ax, 'on'); hold(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
        xlabel(ax, 'N (m)'); ylabel(ax, 'E (m)'); zlabel(ax, 'D (m)');
        title(ax, '3D Scenario');
        plotHandles.axes = ax;
        
        % Pre-compute cone geometry for efficiency
        [Y, Z, X] = cylinder([0 1.5], 12);
        X = X * 5;
        plotHandles.cone_X = X;
        plotHandles.cone_Y = Y;
        plotHandles.cone_Z = Z;
        
        % Add 2D inset axes for image plane (pxbar, pybar) in lower-right corner
        axImg = axes('Parent', hPlotPanel, 'Units', 'normalized', 'Position', [0.70 0.08 0.25 0.25]);
        grid(axImg, 'on'); hold(axImg, 'on'); axis(axImg, 'equal');
        xlabel(axImg, 'p_x'); ylabel(axImg, 'p_y');
        title(axImg, 'Image Plane', 'FontSize', 9);
        xlim(axImg, [-1 1]); ylim(axImg, [-1 1]);
        % Draw origin crosshairs
        plot(axImg, [-1 1], [0 0], 'k--', 'LineWidth', 0.5);
        plot(axImg, [0 0], [-1 1], 'k--', 'LineWidth', 0.5);
        plotHandles.axImg = axImg;
    end

    function setupStateView()
        % Replicating Figure 1 of plot_eskf_results (3x2 grid)
        % Rows: Pos, Vel, Image
        plotHandles.axes = gobjects(3,2);
        titles = {'Relative Position (All Components)', 'Position Estimation Error with 3σ bounds', ...
                  'Relative Velocity (All Components)', 'Velocity Estimation Error with 3σ bounds', ...
                  'Normalized Image Features', 'Image Feature Estimation Error with 3σ bounds'};
        for i=1:3
            for j=1:2
                idx = (i-1)*2 + j;
                ax = subplot(3, 2, idx, 'Parent', hPlotPanel);
                grid(ax, 'on'); hold(ax, 'on');
                title(ax, titles{idx});
                plotHandles.axes(i,j) = ax;
            end
        end
    end
    
    function setupAttView()
        % Replicating Figure 3 (2x3 grid)
        % Row 1: Euler, Row 2: Errors
        plotHandles.axes = gobjects(2,3);
        titles = {'Yaw', 'Pitch', 'Roll', 'Yaw Err', 'Pitch Err', 'Roll Err'};
        for i=1:2
            for j=1:3
                idx = (i-1)*3 + j;
                ax = subplot(2, 3, idx, 'Parent', hPlotPanel);
                grid(ax, 'on'); hold(ax, 'on');
                title(ax, titles{idx});
                plotHandles.axes(i,j) = ax;
            end
        end
    end

    function setupBiasView()
        % Replicating Figure 2 (2x2 grid)
        % Row 1: Gyro, Row 2: Accel (Wait, plot_eskf_results is 2x2 total: Gyro L/R, Accel L/R)
        plotHandles.axes = gobjects(2,2);
        titles = {'Gyro Bias', 'Gyro Bias Err', 'Accel Bias', 'Accel Bias Err'};
        for i=1:2
             for j=1:2
                idx = (i-1)*2 + j;
                ax = subplot(2, 2, idx, 'Parent', hPlotPanel);
                grid(ax, 'on'); hold(ax, 'on');
                title(ax, titles{idx});
                plotHandles.axes(i,j) = ax;
             end
        end
    end

    function updateVisualization()
        if isempty(plotHandles.type)
             changeView([], [], '3D'); 
        end
        
        % Update Stats
        r = panelHandles.results;
        if sim.k > 0
            p_err = norm(sim.x_true(5:7) - sim.x_est(5:7));
            v_err = norm(sim.x_true(8:10) - sim.x_est(8:10));
            r.txt_err_pos.String = sprintf('%.4f m', p_err);
            r.txt_err_vel.String = sprintf('%.4f m/s', v_err);
        end
        
        switch plotHandles.type
            case '3D'
                update3DData();
            case 'STATE'
                updateStateData();
            case 'ATT'
                updateAttData();
            case 'BIAS'
                updateBiasData();
            case 'COV'
                updateCovData();
        end
    end
    
    function update3DData()
        ax = plotHandles.axes;
        cla(ax);
        
        % Guard: Check if simulation has been initialized
        if ~isstruct(sim.history) || ~isfield(sim.history, 'count')
            title(ax, '3D Scenario - Run simulation to see data');
            return;
        end
        
        % Check if history has data for trajectories
        if sim.history.count > 1
            % Plot Interceptor True Trajectory
            pInt = sim.history.p_int(:, 1:sim.history.count);
            hIntPath = plot3(ax, pInt(1,:), pInt(2,:), pInt(3,:), 'k-', 'LineWidth', 3);
            
            % Plot Target True Trajectory
            pTgt = sim.history.p_tgt(:, 1:sim.history.count);
            hTgtPath = plot3(ax, pTgt(1,:), pTgt(2,:), pTgt(3,:), 'b-', 'LineWidth', 1.5);
            
            % Estimated Target Trajectory (p_int - p_r_est)
            pRest = sim.history.x_est(5:7, 1:sim.history.count);
            pTgtEst = pInt - pRest;
            hTgtEstPath = plot3(ax, pTgtEst(1,:), pTgtEst(2,:), pTgtEst(3,:), 'r-', 'LineWidth', 1.5);
        else
            hIntPath = []; hTgtPath = []; hTgtEstPath = [];
        end
        
        % Draw Current Interceptor Cone (True Attitude)
        if isfield(plotHandles, 'cone_X') && ~isempty(sim.x_true)
            q_true = sim.x_true(1:4);
            R_true = quat2rotm(q_true(:)');
            pts = [plotHandles.cone_X(:)'; plotHandles.cone_Y(:)'; plotHandles.cone_Z(:)'];
            pts_rot = R_true * pts + sim.p_int;
            X_rot = reshape(pts_rot(1,:), size(plotHandles.cone_X));
            Y_rot = reshape(pts_rot(2,:), size(plotHandles.cone_Y));
            Z_rot = reshape(pts_rot(3,:), size(plotHandles.cone_Z));
            hConeTrue = surf(ax, X_rot, Y_rot, Z_rot, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.6);
            
            % Estimated Attitude Cone
            q_est = sim.x_est(1:4);
            R_est = quat2rotm(q_est(:)');
            pts_est = R_est * pts + sim.p_int;
            X_est = reshape(pts_est(1,:), size(plotHandles.cone_X));
            Y_est = reshape(pts_est(2,:), size(plotHandles.cone_Y));
            Z_est = reshape(pts_est(3,:), size(plotHandles.cone_Z));
            hConeEst = surf(ax, X_est, Y_est, Z_est, 'FaceColor', 'r', 'EdgeColor', 'r', 'FaceAlpha', 0.15, 'EdgeAlpha', 0.3);
        else
            hConeTrue = plot3(ax, sim.p_int(1), sim.p_int(2), sim.p_int(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            hConeEst = [];
        end
        
        % Draw Target Current Position
        hTgt = plot3(ax, sim.p_tgt(1), sim.p_tgt(2), sim.p_tgt(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        
        title(ax, sprintf('3D Scenario (t=%.2fs)', sim.t));
        view(ax, 3);
        
        % Legend
        if sim.history.count > 1 && ~isempty(hConeEst)
            legend(ax, [hIntPath, hTgtPath, hTgtEstPath, hConeTrue, hConeEst, hTgt], ...
                   {'Int Traj', 'Tgt Traj (True)', 'Tgt Traj (Est)', 'Int Att (True)', 'Int Att (Est)', 'Tgt Pos'}, 'Location', 'best');
        elseif ~isempty(hConeEst)
            legend(ax, [hConeTrue, hConeEst, hTgt], {'Interceptor (True)', 'Interceptor (Est)', 'Target'}, 'Location', 'best');
        else
            legend(ax, [hConeTrue, hTgt], {'Interceptor', 'Target'}, 'Location', 'best');
        end
        
        % Dynamic Axis Limits
        % When simulation is done, show full trajectory; otherwise track current
        if ~appState.isRunning && sim.history.count > 1
            % Zoom out to show full trajectory
            pInt = sim.history.p_int(:, 1:sim.history.count);
            pTgt = sim.history.p_tgt(:, 1:sim.history.count);
            pts_all = [pInt, pTgt];
        else
            % Track current positions
            pts_all = [sim.p_int, sim.p_tgt];
        end
        
        min_xyz = min(pts_all, [], 2);
        max_xyz = max(pts_all, [], 2);
        range_xyz = max_xyz - min_xyz;
        
        % Enforce minimum 20m range on each axis
        min_range = 20;
        for i = 1:3
            if range_xyz(i) < min_range
                center = (min_xyz(i) + max_xyz(i)) / 2;
                min_xyz(i) = center - min_range/2;
                max_xyz(i) = center + min_range/2;
            end
        end
        
        margin = 5; % Extra margin in meters
        xlim(ax, [min_xyz(1) - margin, max_xyz(1) + margin]);
        ylim(ax, [min_xyz(2) - margin, max_xyz(2) + margin]);
        zlim(ax, [min_xyz(3) - margin, max_xyz(3) + margin]);
        
        % Update 2D Image Plane Inset
        if isfield(plotHandles, 'axImg') && isvalid(plotHandles.axImg)
            axImg = plotHandles.axImg;
            % Clear previous data but keep crosshairs (redraw them)
            cla(axImg);
            hold(axImg, 'on');
            plot(axImg, [-1 1], [0 0], 'k--', 'LineWidth', 0.5);
            plot(axImg, [0 0], [-1 1], 'k--', 'LineWidth', 0.5);
            
            % Plot trajectory history
            if sim.history.count > 1
                pbar_true = sim.history.x_true(11:12, 1:sim.history.count);
                pbar_est = sim.history.x_est(11:12, 1:sim.history.count);
                plot(axImg, pbar_true(1,:), pbar_true(2,:), 'b-', 'LineWidth', 1.5);
                plot(axImg, pbar_est(1,:), pbar_est(2,:), 'r--', 'LineWidth', 1.2);
            end
            
            % Plot current position with markers
            if ~isempty(sim.x_true) && ~isempty(sim.x_est)
                plot(axImg, sim.x_true(11), sim.x_true(12), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                plot(axImg, sim.x_est(11), sim.x_est(12), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
            
            legend(axImg, 'True', 'Est', 'Location', 'northeast', 'FontSize', 7);
            title(axImg, 'Image Plane ($\bar{p}$)', 'Interpreter', 'latex', 'FontSize', 9);
            xlim(axImg, [-1 1]); ylim(axImg, [-1 1]);
            grid(axImg, 'on');
        end
    end
    
    function updateStateData()
        if sim.history.count < 1, return; end
        
        t = sim.history.t(1:sim.history.count);
        xt = sim.history.x_true(:, 1:sim.history.count);
        xe = sim.history.x_est(:, 1:sim.history.count);
        Pd = sim.history.P_diag(:, 1:sim.history.count);
        
        % Row 1: Position (Indices 5:7)
        axPos = plotHandles.axes(1,1); cla(axPos);
        plot(axPos, t, xt(5:7, :)', '-', 'LineWidth', 1.5); 
        set(axPos, 'ColorOrderIndex', 1);
        plot(axPos, t, xe(5:7, :)', '--', 'LineWidth', 1.5);
        xlabel(axPos, 'Time (s)'); ylabel(axPos, 'Position (m)');
        legend(axPos, 'Tx','Ty','Tz','Ex','Ey','Ez', 'Location', 'best');
        title(axPos, 'Relative Position');
        
        axPosErr = plotHandles.axes(1,2); cla(axPosErr);
        errP = xt(5:7,:) - xe(5:7,:);
        sigP = sqrt(Pd(4:6,:));
        plot(axPosErr, t, errP', '-', 'LineWidth', 1.5);
        hold(axPosErr, 'on');
        plot(axPosErr, t, 3*sigP(3,:), 'k--', 'LineWidth', 1.5);
        plot(axPosErr, t, -3*sigP(3,:), 'k--', 'LineWidth', 1.5);
        xlabel(axPosErr, 'Time (s)'); ylabel(axPosErr, 'Error (m)');
        legend(axPosErr, 'Err X', 'Err Y', 'Err Z', '±3σ', 'Location', 'best');
        title(axPosErr, 'Position Error with 3σ bounds');
        
        % Row 2: Velocity (Indices 8:10)
        axVel = plotHandles.axes(2,1); cla(axVel);
        plot(axVel, t, xt(8:10, :)', '-', 'LineWidth', 1.5);
        set(axVel, 'ColorOrderIndex', 1);
        plot(axVel, t, xe(8:10, :)', '--', 'LineWidth', 1.5);
        xlabel(axVel, 'Time (s)'); ylabel(axVel, 'Velocity (m/s)');
        legend(axVel, 'Tx','Ty','Tz','Ex','Ey','Ez', 'Location', 'best');
        title(axVel, 'Relative Velocity');
        
        axVelErr = plotHandles.axes(2,2); cla(axVelErr);
        errV = xt(8:10,:) - xe(8:10,:);
        sigV = sqrt(Pd(7:9,:));
        plot(axVelErr, t, errV', '-', 'LineWidth', 1.5);
        hold(axVelErr, 'on');
        plot(axVelErr, t, 3*sigV(3,:), 'k--', 'LineWidth', 1.5);
        plot(axVelErr, t, -3*sigV(3,:), 'k--', 'LineWidth', 1.5);
        xlabel(axVelErr, 'Time (s)'); ylabel(axVelErr, 'Error (m/s)');
        legend(axVelErr, 'Err X', 'Err Y', 'Err Z', '±3σ', 'Location', 'best');
        title(axVelErr, 'Velocity Error with 3σ bounds');
        
        % Row 3: Image (Indices 11:12)
        axImg = plotHandles.axes(3,1); cla(axImg);
        plot(axImg, t, xt(11:12, :)', '-', 'LineWidth', 1.5);
        set(axImg, 'ColorOrderIndex', 1);
        plot(axImg, t, xe(11:12, :)', '--', 'LineWidth', 1.5);
        xlabel(axImg, 'Time (s)'); ylabel(axImg, 'Norm Coord');
        legend(axImg, 'True px', 'True py', 'Est px', 'Est py', 'Location', 'best');
        title(axImg, 'Normalized Image Features');
        
        axImgErr = plotHandles.axes(3,2); cla(axImgErr);
        errI = xt(11:12,:) - xe(11:12,:);
        sigI = sqrt(Pd(10:11,:));
        plot(axImgErr, t, errI', '-', 'LineWidth', 1.5);
        hold(axImgErr, 'on');
        plot(axImgErr, t, 3*sigI(2,:), 'k--', 'LineWidth', 1.5);
        plot(axImgErr, t, -3*sigI(2,:), 'k--', 'LineWidth', 1.5);
        xlabel(axImgErr, 'Time (s)'); ylabel(axImgErr, 'Error');
        legend(axImgErr, 'Err px', 'Err py', '±3σ', 'Location', 'best');
        title(axImgErr, 'Image Error with 3σ bounds');
    end

    function updateAttData()
        if sim.history.count < 1, return; end
        t = sim.history.t(1:sim.history.count);
        xt = sim.history.x_true(:, 1:sim.history.count);
        xe = sim.history.x_est(:, 1:sim.history.count);
        Pd = sim.history.P_diag(:, 1:sim.history.count);
        
        % Convert Quats(1:4) to Euler
        eulT = quat2eul(xt(1:4,:)', 'ZYX')' * 180/pi;
        eulE = quat2eul(xe(1:4,:)', 'ZYX')' * 180/pi;
        
        names = {'Yaw', 'Pitch', 'Roll'};
        % Row 1: Angles
        for i=1:3
            ax = plotHandles.axes(1,i); cla(ax);
            plot(ax, t, eulT(i,:), 'b-', 'LineWidth', 1.2); hold(ax, 'on');
            plot(ax, t, eulE(i,:), 'r--', 'LineWidth', 1.2);
            xlabel(ax, 'Time (s)'); ylabel(ax, [names{i} ' (deg)']);
            title(ax, [names{i} ' Angle']);
            legend(ax, 'True', 'Est', 'Location', 'best');
            grid(ax, 'on');
        end
        
        % Row 2: Errors
        errE = wrapTo180(eulT - eulE);
        sigE = sqrt(Pd(1:3,:)) * 180/pi;
        
        kMap = [3, 2, 1]; % Yaw->3, Pitch->2, Roll->1
        for i=1:3
             ax = plotHandles.axes(2,i); cla(ax);
             k = kMap(i);
             plot(ax, t, errE(i,:), 'b-', 'LineWidth', 1.2); hold(ax, 'on');
             plot(ax, t, 3*sigE(k,:), 'k--', 'LineWidth', 1);
             plot(ax, t, -3*sigE(k,:), 'k--', 'LineWidth', 1);
             xlabel(ax, 'Time (s)'); ylabel(ax, [names{i} ' Err (deg)']);
             title(ax, [names{i} ' Error with 3σ bounds']);
             legend(ax, 'Error', '±3σ', 'Location', 'best');
             grid(ax, 'on');
        end
    end

    function updateBiasData()
        if sim.history.count < 1, return; end
        t = sim.history.t(1:sim.history.count);
        xt = sim.history.x_true(:, 1:sim.history.count);
        xe = sim.history.x_est(:, 1:sim.history.count);
        Pd = sim.history.P_diag(:, 1:sim.history.count);
        
        % Gyro Bias: True 13:15, Err 12:14
        axGy = plotHandles.axes(1,1); cla(axGy);
        plot(axGy, t, xt(13:15,:)', '-', 'LineWidth', 1.2); hold(axGy, 'on');
        set(axGy, 'ColorOrderIndex', 1);
        plot(axGy, t, xe(13:15,:)', '--', 'LineWidth', 1.2);
        xlabel(axGy, 'Time (s)'); ylabel(axGy, 'Gyro Bias (rad/s)');
        title(axGy, 'Gyroscope Bias Estimation');
        legend(axGy, 'Tx','Ty','Tz','Ex','Ey','Ez', 'Location', 'best');
        grid(axGy, 'on');
        
        axGyE = plotHandles.axes(1,2); cla(axGyE);
        errGy = xt(13:15,:)-xe(13:15,:);
        sigGy = sqrt(Pd(12:14,:));
        plot(axGyE, t, errGy', '-', 'LineWidth', 1.2); hold(axGyE, 'on');
        plot(axGyE, t, 3*sigGy(3,:), 'k--');
        plot(axGyE, t, -3*sigGy(3,:), 'k--');
        xlabel(axGyE, 'Time (s)'); ylabel(axGyE, 'Error (rad/s)');
        title(axGyE, 'Gyro Bias Error with 3σ bounds');
        legend(axGyE, 'Err X','Err Y','Err Z','±3σ', 'Location', 'best');
        grid(axGyE, 'on');
        
        % Accel Bias: True 16:18, Err 15:17
        axAc = plotHandles.axes(2,1); cla(axAc);
        plot(axAc, t, xt(16:18,:)', '-', 'LineWidth', 1.2); hold(axAc, 'on');
        set(axAc, 'ColorOrderIndex', 1);
        plot(axAc, t, xe(16:18,:)', '--', 'LineWidth', 1.2);
        xlabel(axAc, 'Time (s)'); ylabel(axAc, 'Accel Bias (m/s^2)');
        title(axAc, 'Accelerometer Bias Estimation');
        legend(axAc, 'Tx','Ty','Tz','Ex','Ey','Ez', 'Location', 'best');
        grid(axAc, 'on');
        
        axAcE = plotHandles.axes(2,2); cla(axAcE);
        errAc = xt(16:18,:)-xe(16:18,:);
        sigAc = sqrt(Pd(15:17,:));
        plot(axAcE, t, errAc', '-', 'LineWidth', 1.2); hold(axAcE, 'on');
        plot(axAcE, t, 3*sigAc(3,:), 'k--');
        plot(axAcE, t, -3*sigAc(3,:), 'k--');
        xlabel(axAcE, 'Time (s)'); ylabel(axAcE, 'Error (m/s^2)');
        title(axAcE, 'Accel Bias Error with 3σ bounds');
        legend(axAcE, 'Err X','Err Y','Err Z','±3σ', 'Location', 'best');
        grid(axAcE, 'on');
    end
    
    function setupCovView()
        % Replicating Figure 4 of plot_eskf_results (2x3 grid)
        plotHandles.axes = gobjects(2,3);
        titles = {'Attitude σ', 'Position σ', 'Velocity σ', 'Image Feature σ', 'Gyro Bias σ', 'Accel Bias σ'};
        for i=1:2
            for j=1:3
                idx = (i-1)*3 + j;
                ax = subplot(2, 3, idx, 'Parent', hPlotPanel);
                grid(ax, 'on'); hold(ax, 'on');
                title(ax, titles{idx});
                plotHandles.axes(i,j) = ax;
            end
        end
    end
    
    function updateCovData()
        if sim.history.count < 1, return; end
        t = sim.history.t(1:sim.history.count);
        Pd = sim.history.P_diag(:, 1:sim.history.count);
        
        % Error state indices: 1-3 att, 4-6 pos, 7-9 vel, 10-11 pbar, 12-14 bgyr, 15-17 bacc
        
        % Attitude σ
        ax = plotHandles.axes(1,1); cla(ax);
        semilogy(ax, t, sqrt(Pd(1,:))*180/pi, 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(2,:))*180/pi, 'g', 'LineWidth', 1.5);
        semilogy(ax, t, sqrt(Pd(3,:))*180/pi, 'b', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ (deg)');
        legend(ax, 'X','Y','Z', 'Location', 'best'); grid(ax, 'on');
        
        % Position σ
        ax = plotHandles.axes(1,2); cla(ax);
        semilogy(ax, t, sqrt(Pd(4,:)), 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(5,:)), 'g', 'LineWidth', 1.5);
        semilogy(ax, t, sqrt(Pd(6,:)), 'b', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ (m)');
        legend(ax, 'X','Y','Z', 'Location', 'best'); grid(ax, 'on');
        
        % Velocity σ
        ax = plotHandles.axes(1,3); cla(ax);
        semilogy(ax, t, sqrt(Pd(7,:)), 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(8,:)), 'g', 'LineWidth', 1.5);
        semilogy(ax, t, sqrt(Pd(9,:)), 'b', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ (m/s)');
        legend(ax, 'X','Y','Z', 'Location', 'best'); grid(ax, 'on');
        
        % Image σ
        ax = plotHandles.axes(2,1); cla(ax);
        semilogy(ax, t, sqrt(Pd(10,:)), 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(11,:)), 'g', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ');
        legend(ax, 'p_x','p_y', 'Location', 'best'); grid(ax, 'on');
        
        % Gyro Bias σ
        ax = plotHandles.axes(2,2); cla(ax);
        semilogy(ax, t, sqrt(Pd(12,:)), 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(13,:)), 'g', 'LineWidth', 1.5);
        semilogy(ax, t, sqrt(Pd(14,:)), 'b', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ (rad/s)');
        legend(ax, 'X','Y','Z', 'Location', 'best'); grid(ax, 'on');
        
        % Accel Bias σ
        ax = plotHandles.axes(2,3); cla(ax);
        semilogy(ax, t, sqrt(Pd(15,:)), 'r', 'LineWidth', 1.5); hold(ax, 'on');
        semilogy(ax, t, sqrt(Pd(16,:)), 'g', 'LineWidth', 1.5);
        semilogy(ax, t, sqrt(Pd(17,:)), 'b', 'LineWidth', 1.5);
        xlabel(ax, 'Time (s)'); ylabel(ax, 'σ (m/s^2)');
        legend(ax, 'X','Y','Z', 'Location', 'best'); grid(ax, 'on');
    end
    
    function onTimerTick(~,~)
        if ~appState.isRunning, return; end
        
        stepsToTake = round((appState.timerPeriod * appState.pace) / sim.config.dt_imu);
        stepsToTake = max(1, stepsToTake);
        
        for k = 1:stepsToTake
            finished = sim.step();
            if finished
                stop(simTimer);
                appState.isRunning = false;
                fprintf('Simulation finished.\n');
                return;
            end
        end
        
        updateVisualization();
    end

    function onLiveToggle(src, ~)
        appState.isLive = src.Value;
    end
    
    function onPaceChange(src, ~)
        val = str2double(src.String);
        if ~isnan(val) && val > 0
            appState.pace = val;
        else
            src.String = num2str(appState.pace);
        end
    end

    function onSave(~,~)
        if sim.k == 0
            msgbox('No simulation results to save.', 'Save Info');
            return;
        end
        
        resDir = fullfile(guiPath, '..', 'results');
        if ~exist(resDir, 'dir'), mkdir(resDir); end
        
        timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        saveFolder = fullfile(resDir, ['sim_' timestamp]);
        mkdir(saveFolder);
        
        results = sim.history;
        config = sim.config;
        save(fullfile(saveFolder, 'simulation_data.mat'), 'results', 'config');
        
        fid = fopen(fullfile(saveFolder, 'config.txt'), 'w');
        fprintf(fid, 'Simulation Configuration\nDate: %s\n\n', timestamp);
        jsonStr = jsonencode(config, 'PrettyPrint', true);
        fprintf(fid, '%s', jsonStr);
        fclose(fid);
        
        savefig(fig, fullfile(saveFolder, 'gui_snapshot.fig'));
        saveas(fig, fullfile(saveFolder, 'gui_snapshot.jpg'));
        
        msgbox(['Results saved to: ' saveFolder], 'Success');
    end
    
    function plotResults()
        % Called after batch run
        assignin('base', 'sim_results', sim.history);
        disp('Results available in base workspace as sim_results');
        updateVisualization();
    end

    function onClose(~,~)
        stop(simTimer);
        delete(simTimer);
        delete(fig);
    end
    
    % Initialize once
    % sim.initialize(); % Defer init to first run or use defaults
    
    % Initialize 3D view at startup so plot area isn't blank
    appState.currentView = ''; % Clear so changeView doesn't return early
    changeView([], [], '3D');
    
    %% Helper: Read Configuration
    function cfg = readConfigFromGUI()
        % Start with current config to preserve unexposed fields (like init_errors)
        cfg = sim.config;
        
        % Model
        m = panelHandles.model;
        cfg.imu_noise.accel_n = str2double(m.ed_acc_n.String);
        cfg.imu_noise.gyro_n = str2double(m.ed_gyr_n.String);
        cfg.imu_noise.accel_w = str2double(m.ed_acc_w.String);
        cfg.imu_noise.gyro_w = str2double(m.ed_gyr_w.String);
        
        imu_hz = str2double(m.ed_imu_hz.String);
        cfg.dt_imu = 1 / imu_hz;
        
        cfg.init_cond.p_int = str2num(m.ed_p_int.String)'; 
        cfg.init_cond.p_tgt = str2num(m.ed_p_tgt.String)'; 
        cfg.init_cond.v_int = str2num(m.ed_v_int.String)';
        cfg.init_cond.v_tgt = str2num(m.ed_v_tgt.String)';
        
        % Initial Euler angles [Yaw, Pitch, Roll] in degrees -> convert to radians
        euler_deg = str2num(m.ed_euler_int.String);
        cfg.init_cond.euler_int = euler_deg(:)' * pi/180; % Store as [yaw, pitch, roll] in radians
        
        % Controller constraints
        cfg.controller.max_velocity = str2double(m.ed_v_max.String);
        cfg.controller.max_omega = str2double(m.ed_w_max.String);
        
        % Sensors
        s = panelHandles.sensors;
        cfg.sensors.useCam = s.chk_cam.Value;
        cfg.dt_image = 1 / str2double(s.ed_cam_hz.String);
        cfg.sensors.sigma_img = str2double(s.ed_cam_noise.String);
        cfg.sensors.delay = str2double(s.ed_cam_delay.String) / 1000; % ms to s
        
        cfg.sensors.useRadar = s.chk_radar.Value;
        cfg.dt_radar = 1 / str2double(s.ed_radar_hz.String);
        cfg.sensors.sigma_radar = str2double(s.ed_radar_noise.String);
        
        % Filter
        f = panelHandles.filter;
        % cfg.filter.type = f.pop_filter.Value; % 1=ESKF
        
        % Fixed DT for ESKF for now (could be adjustable)
        cfg.dt_eskf = 1/100; 
        
        cfg.filter.init_sigma.position = str2double(f.ed_sig_pos.String);
        cfg.filter.init_sigma.velocity = str2double(f.ed_sig_vel.String);
        cfg.filter.init_sigma.attitude = str2double(f.ed_sig_att.String);
        cfg.filter.init_sigma.pbar = str2double(f.ed_sig_pbar.String);
        cfg.filter.init_sigma.b_gyr = str2double(f.ed_sig_bg.String);
        cfg.filter.init_sigma.b_acc = str2double(f.ed_sig_ba.String);
        
        % Initial errors (from new GUI fields)
        cfg.filter.init_errors.position = str2double(f.ed_err_pos.String) * ones(3,1);
        cfg.filter.init_errors.velocity = str2double(f.ed_err_vel.String) * ones(3,1);
        cfg.filter.init_errors.euler_deg = str2double(f.ed_err_att.String) * ones(3,1);
        cfg.filter.init_errors.pbar = str2double(f.ed_err_pbar.String) * ones(2,1);
        cfg.filter.init_errors.b_gyr = str2double(f.ed_err_bg.String) * ones(3,1);
        cfg.filter.init_errors.b_acc = str2double(f.ed_err_ba.String) * ones(3,1);

        % Limit t_total if needed, or leave as default
        % cfg.t_total = 30; 
    end
end
