function buck_converter_gui()
    % Buck Converter Fuzzy PID Control GUI
    % Real-time interface for controlling and monitoring buck converter
    
    % GUI Data Structure
    gui_data = struct();
    
    % Control parameters
    gui_data.is_running = false;
    gui_data.timer_obj = [];
    gui_data.serial_obj = [];
    
    % System parameters
    gui_data.setpoint = 5.0;           % Desired output voltage (V)
    gui_data.R1 = 2200;                % Voltage divider R1 (ohm)
    gui_data.R2 = 1000;                % Voltage divider R2 (ohm)
    gui_data.Vref = 3;               % ADC reference voltage (V)
    gui_data.sampling_time = 0.005;       % Sampling time (s) - Optimized for GUI responsiveness
    gui_data.use_fuzzy = true;         % Use fuzzy PID adaptation
    
    % Manual PID parameters - optimized values
    gui_data.Kp_manual = 0.085;
    gui_data.Ki_manual = 0.7;
    gui_data.Kd_manual = 0.0001;
    
    % Deadband control
    gui_data.deadband_threshold = 0.1;  % Deadband threshold in volts
    
    % Control variables
    gui_data.error_integral = 0;
    gui_data.error_prev = 0;
    gui_data.current_duty = 0;
    gui_data.current_voltage = 0;
    gui_data.current_error = 0;
    gui_data.current_Kp = 0;
    gui_data.current_Ki = 0;
    gui_data.current_Kd = 0;
    
    % Data logging
    gui_data.time_data = [];
    gui_data.voltage_data = [];
    gui_data.error_data = [];
    gui_data.setpoint_data = [];
    gui_data.duty_data = [];
    gui_data.Kp_data = [];
    gui_data.Ki_data = [];
    gui_data.Kd_data = [];
    gui_data.max_data_points = 500;    % Maximum points to display
    
    % Create GUI
    create_gui();
    
    % ====================================================================
    % SERIAL COMMUNICATION FUNCTIONS (Local Functions)
    % ====================================================================
    
    function serial_obj = init_serial_comm()
        % Initialize serial communication with STM32 on COM5
        % Returns serial object for communication
        
        try
            % Create serial object
            serial_obj = serialport("COM5", 921600);
            
            % Configure serial parameters
            configureTerminator(serial_obj, "CR/LF");
            serial_obj.Timeout = 1; % 1 second timeout
            
            % Clear any existing data
            flush(serial_obj);
            
            fprintf('Serial communication initialized on COM5\n');
            
        catch ME
            fprintf('Error initializing serial communication: %s\n', ME.message);
            serial_obj = [];
        end
    end

    function success = send_duty_cycle(serial_obj, duty_cycle)
        % Send duty cycle to STM32
        % duty_cycle: value between 0 and 1
        % Returns: 1 for success, 0 for failure
        
        success = 0;
        
        if isempty(serial_obj)
            fprintf('Serial object not initialized\n');
            return;
        end
        
        try
            % Convert duty cycle to percentage (0-100)
            duty_percent = round(duty_cycle * 100);
            
            % Send command in format "DUTY:XX\n"
            command = sprintf('DUTY:%d\n', duty_percent);
            write(serial_obj, uint8(command), 'uint8');
            
            success = 1;
            
        catch ME
            fprintf('Error sending duty cycle: %s\n', ME.message);
        end
    end

    function adc_value = read_adc_value(serial_obj)
        % Read ADC value from STM32
        % Returns: ADC value (0-4095) or -1 if error
        
        adc_value = -1;
        
        if isempty(serial_obj)
            fprintf('Serial object not initialized\n');
            return;
        end
        
        try
            % Send request for ADC value
            write(serial_obj, uint8('ADC?'), 'uint8');
            write(serial_obj, 10, 'uint8');  % Send newline character (ASCII 10)
            
            % Wait a bit for response
            pause(0.001);
            
            % Read response
            if serial_obj.NumBytesAvailable > 0
                response = readline(serial_obj);
                response = strip(response); % Remove whitespace
                
                % Parse ADC value from response (expected format: "ADC:XXXX")
                if startsWith(response, "ADC:")
                    adc_str = extractAfter(response, "ADC:");
                    adc_value = str2double(adc_str);
                    
                    % Validate ADC value range
                    if isnan(adc_value) || adc_value < 0 || adc_value > 4095
                        adc_value = -1;
                    end
                end
            end
            
        catch ME
            fprintf('Error reading ADC value: %s\n', ME.message);
        end
    end

    function voltage = adc_to_voltage(adc_value, R1, R2, Vref)
        % Convert ADC value to actual voltage considering voltage divider
        % Inputs:
        %   adc_value: ADC reading (0-4095)
        %   R1, R2: voltage divider resistors (R1 to Vout, R2 to GND)
        %   Vref: ADC reference voltage (typically 3.3V)
        % Output:
        %   voltage: actual output voltage
        
        if adc_value < 0 || adc_value > 4095
            voltage = 0;
            return;
        end
        
        % Convert ADC to voltage at ADC pin
        adc_voltage = (adc_value / 4095) * Vref;
        
        % Calculate actual voltage considering voltage divider
        % Vout = Vadc * (R1 + R2) / R2
        voltage = adc_voltage * (R1 + R2) / R2;
    end

    % ====================================================================
    % GUI CREATION AND CALLBACK FUNCTIONS
    % ====================================================================
    
    function create_gui()
        % Create main figure
        gui_data.fig = figure('Name', 'Buck Converter Fuzzy PID Control', ...
                              'Position', [100, 100, 1200, 800], ...
                              'NumberTitle', 'off', ...
                              'MenuBar', 'none', ...
                              'ToolBar', 'none', ...
                              'Resize', 'off', ...
                              'CloseRequestFcn', @close_gui);
        
        % Create main panels
        create_control_panel();
        create_monitoring_panel();
        create_graph_panels();
        
        % Initialize GUI state
        update_gui_state();
    end
    
    function create_control_panel()
        % Control panel
        control_panel = uipanel('Parent', gui_data.fig, ...
                               'Title', 'Control Parameters', ...
                               'Position', [0.02, 0.7, 0.25, 0.28]);
        
        % Setpoint control
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Setpoint (V):', 'Position', [10, 180, 80, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.setpoint_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                          'String', num2str(gui_data.setpoint), ...
                                          'Position', [100, 180, 60, 25], ...
                                          'Callback', @setpoint_callback);
        
        % Fuzzy/Manual mode
        gui_data.fuzzy_checkbox = uicontrol('Parent', control_panel, 'Style', 'checkbox', ...
                                           'String', 'Use Fuzzy PID', ...
                                           'Value', gui_data.use_fuzzy, ...
                                           'Position', [10, 150, 120, 20], ...
                                           'Callback', @fuzzy_mode_callback);
        
        % Manual PID parameters
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Manual PID:', 'Position', [10, 120, 80, 20], ...
                  'HorizontalAlignment', 'left');
        
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Kp:', 'Position', [10, 95, 25, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.Kp_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                    'String', num2str(gui_data.Kp_manual), ...
                                    'Position', [40, 95, 50, 20], ...
                                    'Callback', @manual_pid_callback);
        
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Ki:', 'Position', [100, 95, 25, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.Ki_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                    'String', num2str(gui_data.Ki_manual), ...
                                    'Position', [125, 95, 50, 20], ...
                                    'Callback', @manual_pid_callback);
        
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Kd:', 'Position', [185, 95, 25, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.Kd_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                    'String', num2str(gui_data.Kd_manual), ...
                                    'Position', [210, 95, 50, 20], ...
                                    'Callback', @manual_pid_callback);
        
        % Voltage divider settings
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Voltage Divider:', 'Position', [10, 65, 100, 20], ...
                  'HorizontalAlignment', 'left');
        
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'R1(Ω):', 'Position', [10, 40, 40, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.R1_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                    'String', num2str(gui_data.R1), ...
                                    'Position', [55, 40, 60, 20], ...
                                    'Callback', @voltage_divider_callback);
        
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'R2(Ω):', 'Position', [125, 40, 40, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.R2_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                    'String', num2str(gui_data.R2), ...
                                    'Position', [170, 40, 60, 20], ...
                                    'Callback', @voltage_divider_callback);
        
        % Deadband control
        uicontrol('Parent', control_panel, 'Style', 'text', ...
                  'String', 'Deadband (V):', 'Position', [10, 15, 80, 20], ...
                  'HorizontalAlignment', 'left');
        gui_data.deadband_edit = uicontrol('Parent', control_panel, 'Style', 'edit', ...
                                          'String', num2str(gui_data.deadband_threshold), ...
                                          'Position', [95, 15, 50, 20], ...
                                          'Callback', @deadband_callback);
        
        % Control buttons - moved up to make room for deadband
        gui_data.start_button = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                                         'String', 'Start', ...
                                         'Position', [160, 15, 50, 20], ...
                                         'BackgroundColor', [0.2, 0.8, 0.2], ...
                                         'FontSize', 8, ...
                                         'Callback', @start_callback);
        
        gui_data.stop_button = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                                        'String', 'STOP', ...
                                        'Position', [220, 15, 50, 20], ...
                                        'BackgroundColor', [0.8, 0.2, 0.2], ...
                                        'ForegroundColor', [1, 1, 1], ...
                                        'FontWeight', 'bold', ...
                                        'FontSize', 8, ...
                                        'Enable', 'off', ...
                                        'Callback', @emergency_stop_callback);
        
        gui_data.clear_button = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                                         'String', 'Clear Data', ...
                                         'Position', [190, 40, 80, 20], ...
                                         'FontSize', 8, ...
                                         'Callback', @clear_data_callback);
    end
    
    function create_monitoring_panel()
        % Monitoring panel
        monitor_panel = uipanel('Parent', gui_data.fig, ...
                               'Title', 'Current Status', ...
                               'Position', [0.02, 0.52, 0.25, 0.16]);
        
        % Current values display
        gui_data.voltage_text = uicontrol('Parent', monitor_panel, 'Style', 'text', ...
                                         'String', 'Output Voltage: 0.00 V', ...
                                         'Position', [10, 80, 150, 20], ...
                                         'HorizontalAlignment', 'left', ...
                                         'FontSize', 10, 'FontWeight', 'bold');
        
        gui_data.error_text = uicontrol('Parent', monitor_panel, 'Style', 'text', ...
                                       'String', 'Error: 0.00 V', ...
                                       'Position', [10, 60, 150, 20], ...
                                       'HorizontalAlignment', 'left');
        
        gui_data.duty_text = uicontrol('Parent', monitor_panel, 'Style', 'text', ...
                                      'String', 'Duty Cycle: 0.0%', ...
                                      'Position', [10, 40, 150, 20], ...
                                      'HorizontalAlignment', 'left');
        
        gui_data.pid_text = uicontrol('Parent', monitor_panel, 'Style', 'text', ...
                                     'String', sprintf('PID: Kp=%.2f Ki=%.2f Kd=%.3f', 0, 0, 0), ...
                                     'Position', [10, 20, 250, 20], ...
                                     'HorizontalAlignment', 'left');
        
        gui_data.status_text = uicontrol('Parent', monitor_panel, 'Style', 'text', ...
                                        'String', 'Status: Stopped', ...
                                        'Position', [10, 0, 200, 20], ...
                                        'HorizontalAlignment', 'left', ...
                                        'ForegroundColor', [0.8, 0.2, 0.2]);
    end
    
    function create_graph_panels()
        % Voltage graph
        voltage_panel = uipanel('Parent', gui_data.fig, ...
                               'Title', 'Output Voltage', ...
                               'Position', [0.3, 0.52, 0.67, 0.46]);
        
        gui_data.voltage_axes = axes('Parent', voltage_panel, ...
                                    'Position', [0.1, 0.15, 0.85, 0.75]);
        xlabel(gui_data.voltage_axes, 'Time (s)');
        ylabel(gui_data.voltage_axes, 'Voltage (V)');
        title(gui_data.voltage_axes, 'Output Voltage vs Setpoint');
        grid(gui_data.voltage_axes, 'on');
        hold(gui_data.voltage_axes, 'on');
        
        % Error graph
        error_panel = uipanel('Parent', gui_data.fig, ...
                             'Title', 'Control Error', ...
                             'Position', [0.3, 0.02, 0.67, 0.48]);
        
        gui_data.error_axes = axes('Parent', error_panel, ...
                                  'Position', [0.1, 0.15, 0.85, 0.75]);
        xlabel(gui_data.error_axes, 'Time (s)');
        ylabel(gui_data.error_axes, 'Error (V)');
        title(gui_data.error_axes, 'Control Error and PID Parameters');
        grid(gui_data.error_axes, 'on');
        hold(gui_data.error_axes, 'on');
    end
    
    function setpoint_callback(~, ~)
        new_setpoint = str2double(get(gui_data.setpoint_edit, 'String'));
        if ~isnan(new_setpoint) && new_setpoint > 0 && new_setpoint <= 20
            gui_data.setpoint = new_setpoint;
            gui_data.error_integral = 0;
            gui_data.error_prev = 0;
            fprintf('Setpoint changed to: %.2f V\n', new_setpoint);
        else
            set(gui_data.setpoint_edit, 'String', num2str(gui_data.setpoint));
            msgbox('Please enter a valid setpoint between 0 and 20V', 'Invalid Input', 'warn');
        end
        drawnow limitrate; % Ensure GUI updates immediately
    end
    
    function fuzzy_mode_callback(~, ~)
        gui_data.use_fuzzy = get(gui_data.fuzzy_checkbox, 'Value');
        if gui_data.use_fuzzy
            fprintf('Fuzzy mode: ON\n');
        else
            fprintf('Fuzzy mode: OFF\n');
        end
        update_gui_state();
        drawnow limitrate; % Ensure GUI updates immediately
    end
    
    function manual_pid_callback(~, ~)
        Kp = str2double(get(gui_data.Kp_edit, 'String'));
        Ki = str2double(get(gui_data.Ki_edit, 'String'));
        Kd = str2double(get(gui_data.Kd_edit, 'String'));
        
        if ~isnan(Kp) && Kp >= 0
            gui_data.Kp_manual = Kp;
        else
            set(gui_data.Kp_edit, 'String', num2str(gui_data.Kp_manual));
        end
        
        if ~isnan(Ki) && Ki >= 0
            gui_data.Ki_manual = Ki;
        else
            set(gui_data.Ki_edit, 'String', num2str(gui_data.Ki_manual));
        end
        
        if ~isnan(Kd) && Kd >= 0
            gui_data.Kd_manual = Kd;
        else
            set(gui_data.Kd_edit, 'String', num2str(gui_data.Kd_manual));
        end
        
        fprintf('Manual PID updated: Kp=%.3f, Ki=%.2f, Kd=%.5f\n', ...
                gui_data.Kp_manual, gui_data.Ki_manual, gui_data.Kd_manual);
        drawnow limitrate; % Ensure GUI updates immediately
    end
    
    function voltage_divider_callback(~, ~)
        R1 = str2double(get(gui_data.R1_edit, 'String'));
        R2 = str2double(get(gui_data.R2_edit, 'String'));
        
        if ~isnan(R1) && R1 > 0
            gui_data.R1 = R1;
        else
            set(gui_data.R1_edit, 'String', num2str(gui_data.R1));
        end
        
        if ~isnan(R2) && R2 > 0
            gui_data.R2 = R2;
        else
            set(gui_data.R2_edit, 'String', num2str(gui_data.R2));
        end
    end
    
    function start_callback(~, ~)
        % Initialize serial communication
        gui_data.serial_obj = init_serial_comm();
        
        if isempty(gui_data.serial_obj)
            msgbox('Failed to initialize serial communication on COM5', 'Connection Error', 'error');
            return;
        end
        
        % Reset control variables
        gui_data.error_integral = 0;
        gui_data.error_prev = 0;
        
        % Start control timer with optimized settings for GUI responsiveness
        gui_data.timer_obj = timer('ExecutionMode', 'fixedSpacing', ...  % fixedSpacing instead of fixedRate
                                  'Period', gui_data.sampling_time, ...
                                  'TimerFcn', @control_loop_callback, ...
                                  'BusyMode', 'drop');  % Drop callbacks if previous one still running
        start(gui_data.timer_obj);
        
        gui_data.is_running = true;
        update_gui_state();
    end
    
    function deadband_callback(~, ~)
        deadband = str2double(get(gui_data.deadband_edit, 'String'));
        
        if ~isnan(deadband) && deadband >= 0 && deadband <= 1.0
            gui_data.deadband_threshold = deadband;
            fprintf('Deadband threshold set to: %.3f V\n', deadband);
        else
            set(gui_data.deadband_edit, 'String', num2str(gui_data.deadband_threshold));
            msgbox('Please enter a valid deadband between 0 and 1.0V', 'Invalid Input', 'warn');
        end
        drawnow limitrate; % Ensure GUI updates immediately
    end
    
    function emergency_stop_callback(~, ~)
        % Disable button immediately to prevent spamming
        set(gui_data.stop_button, 'Enable', 'off');
        set(gui_data.stop_button, 'String', 'STOPPING...');
        drawnow; % Force GUI update
        
        % Call the actual stop function
        stop_callback();
        
        % Re-enable button after a brief delay (if still running somehow)
        pause(0.5);
        if gui_data.is_running
            set(gui_data.stop_button, 'Enable', 'on');
            set(gui_data.stop_button, 'String', 'STOP');
        end
    end
    
    function stop_callback(~, ~)
        fprintf("Emergency Stop Activated\n");
        
        % EMERGENCY STOP - Force immediate stop
        gui_data.is_running = false;
        
        % Force stop timer with error handling
        try
            % First, try to stop the timer gracefully
            if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
                if strcmp(get(gui_data.timer_obj, 'Running'), 'on')
                    stop(gui_data.timer_obj);
                    % Wait a bit for timer to stop
                    pause(0.05);
                end
                delete(gui_data.timer_obj);
            end
        catch ME
            fprintf('Timer stop error (forced cleanup): %s\n', ME.message);
        end
        gui_data.timer_obj = [];
        
        % Emergency duty cycle shutdown - multiple attempts
        emergency_shutdown_attempts = 3;
        for attempt = 1:emergency_shutdown_attempts
            try
                if ~isempty(gui_data.serial_obj) && isvalid(gui_data.serial_obj)
                    % Send zero duty cycle with reduced timeout for emergency stop
                    write(gui_data.serial_obj, uint8('DUTY:0'), 'uint8');
                    write(gui_data.serial_obj, 10, 'uint8');  % Newline
                    pause(0.01);  % Brief pause
                    
                    % Verify the command was sent
                    fprintf('Emergency duty cycle 0%% sent (attempt %d)\n', attempt);
                    break;  % Success, exit loop
                end
            catch ME
                fprintf('Emergency duty cycle send failed (attempt %d): %s\n', attempt, ME.message);
                if attempt == emergency_shutdown_attempts
                    fprintf('WARNING: Could not send emergency stop signal to STM32!\n');
                end
            end
        end
        
        % Force close serial communication
        try
            if ~isempty(gui_data.serial_obj)
                % Try graceful close first
                try
                    flush(gui_data.serial_obj);
                catch
                    % Ignore flush errors during emergency stop
                end
                
                delete(gui_data.serial_obj);
                fprintf('Serial communication closed\n');
            end
        catch ME
            fprintf('Serial close error (forced cleanup): %s\n', ME.message);
        end
        gui_data.serial_obj = [];
        
        % Reset all control variables immediately
        gui_data.error_integral = 0;
        gui_data.error_prev = 0;
        gui_data.current_duty = 0;
        gui_data.current_voltage = 0;
        gui_data.current_error = 0;
        
        % Force GUI update
        try
            update_gui_state();
            % Update monitoring display to show stopped state
            set(gui_data.voltage_text, 'String', 'Output Voltage: -- V');
            set(gui_data.error_text, 'String', 'Error: -- V');
            set(gui_data.duty_text, 'String', 'Duty Cycle: 0.0%');
            set(gui_data.pid_text, 'String', 'PID: Kp=0.000 Ki=0.00 Kd=0.00000');
        catch ME
            fprintf('GUI update error during emergency stop: %s\n', ME.message);
        end
        
        fprintf("Emergency Stop Complete - All systems stopped\n");
        
        % ====================================================================
        % FUZZY PID ANALYSIS AND VISUALIZATION
        % ====================================================================
        
        fprintf("\n=== STARTING FUZZY PID CONTROLLER ANALYSIS ===\n");
        
        try
            % 1. Calculate Control System Performance Metrics
            calculate_performance_metrics();
            
            % 2. Define and Display Fuzzy PID Inference System
            display_fuzzy_inference_system();
            
            % 3. Plot Membership Functions
            plot_membership_functions();
            
            % 4. Display Rule Tables
            display_rule_tables();
            
            % 5. Show Fuzzy Rule Viewer
            show_fuzzy_rule_viewer();
            
            fprintf("=== FUZZY PID CONTROLLER ANALYSIS COMPLETED ===\n\n");
            
        catch ME
            fprintf('Error during fuzzy PID analysis: %s\n', ME.message);
        end
    end
    
    function calculate_performance_metrics()
        fprintf("\n--- CALCULATING PERFORMANCE METRICS ---\n");
        
        if length(gui_data.time_data) < 10
            fprintf("Insufficient data for performance analysis (need at least 10 points)\n");
            return;
        end
        
        try
            % Extract data
            time = gui_data.time_data;
            voltage = gui_data.voltage_data;
            setpoint = gui_data.setpoint_data;
            
            % Find step response (look for setpoint changes)
            setpoint_changes = find(abs(diff(setpoint)) > 0.1);
            
            if isempty(setpoint_changes)
                % Use the entire response as step response
                step_start_idx = 1;
                final_setpoint = setpoint(end);
            else
                % Use the last setpoint change
                step_start_idx = setpoint_changes(end);
                final_setpoint = setpoint(end);
            end
            
            % Extract step response data
            step_time = time(step_start_idx:end) - time(step_start_idx);
            step_voltage = voltage(step_start_idx:end);
            
            % Calculate metrics
            metrics = struct();
            
            % 1. Rise Time (10% to 90% of final value)
            final_value = final_setpoint;
            value_10 = 0.1 * final_value;
            value_90 = 0.9 * final_value;
            
            idx_10 = find(step_voltage >= value_10, 1);
            idx_90 = find(step_voltage >= value_90, 1);
            
            if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
                metrics.rise_time = step_time(idx_90) - step_time(idx_10);
            else
                metrics.rise_time = NaN;
            end
            
            % 2. Maximum Overshoot
            max_voltage = max(step_voltage);
            if max_voltage > final_value
                metrics.max_overshoot = ((max_voltage - final_value) / final_value) * 100;
            else
                metrics.max_overshoot = 0;
            end
            
            % 3. Settling Time (2% criterion)
            settling_band = 0.02 * final_value;
            settling_indices = find(abs(step_voltage - final_value) <= settling_band);
            
            if ~isempty(settling_indices)
                % Find last time voltage went outside settling band
                for i = length(step_voltage):-1:1
                    if abs(step_voltage(i) - final_value) > settling_band
                        if i < length(step_voltage)
                            metrics.settling_time = step_time(i+1);
                        else
                            metrics.settling_time = step_time(end);
                        end
                        break;
                    end
                end
                if ~isfield(metrics, 'settling_time')
                    metrics.settling_time = step_time(settling_indices(1));
                end
            else
                metrics.settling_time = NaN;
            end
            
            % 4. Steady State Error
            if length(step_voltage) >= 10
                steady_state_value = mean(step_voltage(end-9:end));  % Average of last 10 points
                metrics.steady_state_error = abs(final_setpoint - steady_state_value);
                metrics.steady_state_error_percent = (metrics.steady_state_error / final_setpoint) * 100;
            else
                metrics.steady_state_error = abs(final_setpoint - step_voltage(end));
                metrics.steady_state_error_percent = (metrics.steady_state_error / final_setpoint) * 100;
            end
            
            % Display results
            fprintf("PERFORMANCE METRICS:\n");
            fprintf("  Rise Time: %.3f s\n", metrics.rise_time);
            fprintf("  Maximum Overshoot: %.2f%%\n", metrics.max_overshoot);
            fprintf("  Settling Time: %.3f s\n", metrics.settling_time);
            fprintf("  Steady State Error: %.4f V (%.2f%%)\n", ...
                   metrics.steady_state_error, metrics.steady_state_error_percent);
            
            % Create performance metrics figure
            figure('Name', 'Control System Performance Metrics', 'Position', [100, 100, 800, 600]);
            
            subplot(2,1,1);
            plot(step_time, step_voltage, 'b-', 'LineWidth', 2);
            hold on;
            plot([0 step_time(end)], [final_setpoint final_setpoint], 'r--', 'LineWidth', 1.5);
            
            % Mark rise time
            if ~isnan(metrics.rise_time) && ~isempty(idx_10) && ~isempty(idx_90)
                plot([step_time(idx_10) step_time(idx_90)], [value_10 value_90], 'go-', 'LineWidth', 2);
                text(step_time(idx_10), value_10-0.2, sprintf('Rise Time: %.3fs', metrics.rise_time), ...
                     'FontSize', 10, 'BackgroundColor', 'white');
            end
            
            % Mark overshoot
            if metrics.max_overshoot > 0
                [~, max_idx] = max(step_voltage);
                plot(step_time(max_idx), step_voltage(max_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
                text(step_time(max_idx), step_voltage(max_idx)+0.1, sprintf('Overshoot: %.2f%%', metrics.max_overshoot), ...
                     'FontSize', 10, 'BackgroundColor', 'white');
            end
            
            % Mark settling time
            if ~isnan(metrics.settling_time)
                plot([metrics.settling_time metrics.settling_time], [0 max(step_voltage)], 'g--', 'LineWidth', 1);
                text(metrics.settling_time+0.1, max(step_voltage)*0.8, sprintf('Settling Time: %.3fs', metrics.settling_time), ...
                     'FontSize', 10, 'BackgroundColor', 'white');
            end
            
            xlabel('Time (s)');
            ylabel('Voltage (V)');
            title('Step Response Analysis');
            grid on;
            legend('Response', 'Setpoint', 'Rise Time', 'Location', 'best');
            
            % Performance summary
            subplot(2,1,2);
            axis off;
            text(0.1, 0.8, 'PERFORMANCE METRICS SUMMARY:', 'FontSize', 14, 'FontWeight', 'bold');
            text(0.1, 0.6, sprintf('Rise Time: %.3f s', metrics.rise_time), 'FontSize', 12);
            text(0.1, 0.5, sprintf('Maximum Overshoot: %.2f%%', metrics.max_overshoot), 'FontSize', 12);
            text(0.1, 0.4, sprintf('Settling Time: %.3f s', metrics.settling_time), 'FontSize', 12);
            text(0.1, 0.3, sprintf('Steady State Error: %.4f V (%.2f%%)', ...
                   metrics.steady_state_error, metrics.steady_state_error_percent), 'FontSize', 12);
            
        catch ME
            fprintf('Error calculating performance metrics: %s\n', ME.message);
        end
    end
    
    function display_fuzzy_inference_system()
        fprintf("\n--- FUZZY PID INFERENCE SYSTEM STAGES ---\n");
        
        try
            % Create the fuzzy systems for display
            [fis_Kp, ~, ~] = get_fuzzy_systems();
            
            fprintf("FUZZY PID CONTROLLER INFERENCE SYSTEM:\n\n");
            
            fprintf("1. FUZZIFICATION STAGE:\n");
            fprintf("   - Input 1: Error (normalized to [-1, 1])\n");
            fprintf("     * Linguistic Variables: NL, NS, Z, PS, PL\n");
            fprintf("     * Membership Functions: Trapezoidal and Triangular\n");
            fprintf("   - Input 2: Error Rate (normalized to [-1, 1])\n");
            fprintf("     * Linguistic Variables: NL, NS, Z, PS, PL\n");
            fprintf("     * Membership Functions: Trapezoidal and Triangular\n\n");
            
            fprintf("2. INFERENCE STAGE:\n");
            fprintf("   - Method: Mamdani Inference\n");
            fprintf("   - Rule Base: 25 rules for each parameter (Kp, Ki, Kd)\n");
            fprintf("   - AND Operation: Minimum\n");
            fprintf("   - OR Operation: Maximum\n");
            fprintf("   - Implication: Minimum\n");
            fprintf("   - Aggregation: Maximum\n\n");
            
            fprintf("3. DEFUZZIFICATION STAGE:\n");
            fprintf("   - Method: Centroid (Center of Area)\n");
            fprintf("   - Output Ranges:\n");
            fprintf("     * Kp: [0.07, 0.08] (tight range around optimal 0.075)\n");
            fprintf("     * Ki: [0.2, 0.3] (tight range around optimal 0.25)\n");
            fprintf("     * Kd: [0.0001, 0.0005] (tight range around optimal 0.0001)\n\n");
            
            fprintf("4. PARAMETER ADAPTATION:\n");
            fprintf("   - Real-time adaptation based on error and error rate\n");
            fprintf("   - Bounded output to maintain system stability\n");
            fprintf("   - Optimized for buck converter dynamics\n\n");
            
            % Create inference system visualization figure
            figure('Name', 'Fuzzy PID Inference System Structure', 'Position', [150, 150, 1000, 700]);
            
            % Create a flowchart representation
            axes('Position', [0.05 0.05 0.9 0.9]);
            axis([0 10 0 10]);
            axis off;
            
            % Title
            text(5, 9.5, 'FUZZY PID CONTROLLER INFERENCE SYSTEM', ...
                 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            
            % Input stage
            rectangle('Position', [0.5 7.5 2 1], 'FaceColor', [0.8 0.9 1], 'EdgeColor', 'black', 'LineWidth', 2);
            text(1.5, 8, 'INPUTS', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(1.5, 7.7, 'Error & Error Rate', 'FontSize', 10, 'HorizontalAlignment', 'center');
            
            % Fuzzification stage
            rectangle('Position', [3.5 7.5 2 1], 'FaceColor', [1 0.9 0.8], 'EdgeColor', 'black', 'LineWidth', 2);
            text(4.5, 8, 'FUZZIFICATION', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(4.5, 7.7, 'MF: NL,NS,Z,PS,PL', 'FontSize', 10, 'HorizontalAlignment', 'center');
            
            % Rule base
            rectangle('Position', [6.5 7.5 2 1], 'FaceColor', [0.9 1 0.8], 'EdgeColor', 'black', 'LineWidth', 2);
            text(7.5, 8, 'RULE BASE', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(7.5, 7.7, '25 Rules Each', 'FontSize', 10, 'HorizontalAlignment', 'center');
            
            % Inference engines (3 parallel paths)
            y_positions = [6, 4.5, 3];
            labels = {'Kp Controller', 'Ki Controller', 'Kd Controller'};
            ranges = {'[0.07-0.08]', '[0.2-0.3]', '[0.0001-0.0005]'};
            
            for i = 1:3
                % Inference
                rectangle('Position', [1.5 y_positions(i) 2 1], 'FaceColor', [1 0.8 0.9], 'EdgeColor', 'black', 'LineWidth', 2);
                text(2.5, y_positions(i)+0.3, 'INFERENCE', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
                text(2.5, y_positions(i)+0.1, labels{i}, 'FontSize', 9, 'HorizontalAlignment', 'center');
                
                % Defuzzification
                rectangle('Position', [4.5 y_positions(i) 2 1], 'FaceColor', [0.9 0.8 1], 'EdgeColor', 'black', 'LineWidth', 2);
                text(5.5, y_positions(i)+0.3, 'DEFUZZIFICATION', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
                text(5.5, y_positions(i)+0.1, 'Centroid', 'FontSize', 9, 'HorizontalAlignment', 'center');
                
                % Output
                rectangle('Position', [7.5 y_positions(i) 1.5 1], 'FaceColor', [0.8 1 0.9], 'EdgeColor', 'black', 'LineWidth', 2);
                text(8.25, y_positions(i)+0.3, labels{i}(1:2), 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
                text(8.25, y_positions(i)+0.1, ranges{i}, 'FontSize', 8, 'HorizontalAlignment', 'center');
            end
            
            % Draw arrows
            % Input to fuzzification
            arrow([2.5 8], [3.5 8], 'LineWidth', 2);
            % Fuzzification to rule base
            arrow([5.5 8], [6.5 8], 'LineWidth', 2);
            
            % Rule base to inference engines
            for i = 1:3
                arrow([7.5 7.5], [2.5 y_positions(i)+0.5], 'LineWidth', 1.5);
                % Inference to defuzzification
                arrow([3.5 y_positions(i)+0.5], [4.5 y_positions(i)+0.5], 'LineWidth', 1.5);
                % Defuzzification to output
                arrow([6.5 y_positions(i)+0.5], [7.5 y_positions(i)+0.5], 'LineWidth', 1.5);
            end
            
            % Final PID output
            rectangle('Position', [4 1 2 1], 'FaceColor', [1 1 0.8], 'EdgeColor', 'black', 'LineWidth', 3);
            text(5, 1.5, 'PID OUTPUT', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            text(5, 1.2, 'u(t) = Kp*e + Ki*∫e + Kd*de/dt', 'FontSize', 10, 'HorizontalAlignment', 'center');
            
            % Arrows to final output
            for i = 1:3
                arrow([8.25 y_positions(i)], [5 2], 'LineWidth', 2);
            end
            
        catch ME
            fprintf('Error displaying fuzzy inference system: %s\n', ME.message);
        end
    end
    
    function plot_membership_functions()
        fprintf("\n--- PLOTTING MEMBERSHIP FUNCTIONS ---\n");
        
        try
            % Get fuzzy systems
            [fis_Kp, fis_Ki, fis_Kd] = get_fuzzy_systems();
            
            % Create membership function plots
            figure('Name', 'Membership Functions - Error and Error Rate', 'Position', [200, 200, 1200, 800]);
            
            % Plot Error membership functions
            subplot(2,3,1);
            plotmf(fis_Kp, 'input', 1);
            title('Error Membership Functions');
            xlabel('Error (normalized)');
            ylabel('Membership Degree');
            grid on;
            
            % Plot Error Rate membership functions
            subplot(2,3,2);
            plotmf(fis_Kp, 'input', 2);
            title('Error Rate Membership Functions');
            xlabel('Error Rate (normalized)');
            ylabel('Membership Degree');
            grid on;
            
            % Plot Kp membership functions
            subplot(2,3,4);
            plotmf(fis_Kp, 'output', 1);
            title('Kp Membership Functions');
            xlabel('Kp Value');
            ylabel('Membership Degree');
            grid on;
            
            % Plot Ki membership functions
            subplot(2,3,5);
            plotmf(fis_Ki, 'output', 1);
            title('Ki Membership Functions');
            xlabel('Ki Value');
            ylabel('Membership Degree');
            grid on;
            
            % Plot Kd membership functions
            subplot(2,3,6);
            plotmf(fis_Kd, 'output', 1);
            title('Kd Membership Functions');
            xlabel('Kd Value');
            ylabel('Membership Degree');
            grid on;
            
            % Add overall title
            sgtitle('Fuzzy PID Controller Membership Functions', 'FontSize', 16, 'FontWeight', 'bold');
            
            fprintf("Membership function plots created successfully.\n");
            
        catch ME
            fprintf('Error plotting membership functions: %s\n', ME.message);
        end
    end
    
    function display_rule_tables()
        fprintf("\n--- DISPLAYING RULE TABLES ---\n");
        
        try
            % Get fuzzy systems
            [fis_Kp, fis_Ki, fis_Kd] = get_fuzzy_systems();
            
            % Create rule table figure
            figure('Name', 'Fuzzy PID Rule Tables', 'Position', [250, 250, 1400, 900]);
            
            % Define linguistic variable names
            error_labels = {'NL', 'NS', 'Z', 'PS', 'PL'};
            error_rate_labels = {'NL', 'NS', 'Z', 'PS', 'PL'};
            
            % Kp output labels
            kp_labels = {'VS', 'S', 'M', 'L', 'VL'};
            ki_labels = {'VS', 'S', 'M', 'L', 'VL'};
            kd_labels = {'VS', 'S', 'M', 'L', 'VL'};
            
            % Create Kp rule table
            subplot(2,2,1);
            kp_rules = get_rule_matrix(fis_Kp);
            display_rule_table_plot(kp_rules, error_labels, error_rate_labels, kp_labels, 'Kp Rules');
            
            % Create Ki rule table
            subplot(2,2,2);
            ki_rules = get_rule_matrix(fis_Ki);
            display_rule_table_plot(ki_rules, error_labels, error_rate_labels, ki_labels, 'Ki Rules');
            
            % Create Kd rule table
            subplot(2,2,3);
            kd_rules = get_rule_matrix(fis_Kd);
            display_rule_table_plot(kd_rules, error_labels, error_rate_labels, kd_labels, 'Kd Rules');
            
            % Create rule table summary in text form
            subplot(2,2,4);
            axis off;
            text(0.05, 0.95, 'RULE TABLE SUMMARY', 'FontSize', 14, 'FontWeight', 'bold');
            
            rule_summary = {
                'Linguistic Variables:',
                '  Error: NL=Negative Large, NS=Negative Small,',
                '         Z=Zero, PS=Positive Small, PL=Positive Large',
                '',
                '  Output: VS=Very Small, S=Small, M=Medium,',
                '          L=Large, VL=Very Large',
                '',
                'Rule Format:',
                '  IF Error is X AND ErrorRate is Y THEN Output is Z',
                '',
                'Total Rules: 25 per parameter (75 total)',
                'Inference Method: Mamdani',
                'Defuzzification: Centroid'
            };
            
            for i = 1:length(rule_summary)
                text(0.05, 0.9 - (i-1)*0.06, rule_summary{i}, 'FontSize', 10);
            end
            
            sgtitle('Fuzzy PID Controller Rule Tables', 'FontSize', 16, 'FontWeight', 'bold');
            
            % Print rule tables to console
            fprintf("\n=== KP RULE TABLE ===\n");
            print_rule_table_console(kp_rules, error_labels, error_rate_labels, kp_labels);
            
            fprintf("\n=== KI RULE TABLE ===\n");
            print_rule_table_console(ki_rules, error_labels, error_rate_labels, ki_labels);
            
            fprintf("\n=== KD RULE TABLE ===\n");
            print_rule_table_console(kd_rules, error_labels, error_rate_labels, kd_labels);
            
        catch ME
            fprintf('Error displaying rule tables: %s\n', ME.message);
        end
    end
    
    function show_fuzzy_rule_viewer()
        fprintf("\n--- SHOWING FUZZY RULE VIEWER ---\n");
        
        try
            % Get fuzzy systems
            [fis_Kp, fis_Ki, fis_Kd] = get_fuzzy_systems();
            
            fprintf("Opening Fuzzy Rule Viewers...\n");
            fprintf("Note: Rule viewers will open in separate windows.\n");
            
            % Open rule viewer for Kp
            try
                ruleview(fis_Kp);
                fprintf("Kp rule viewer opened successfully.\n");
            catch ME
                fprintf('Error opening Kp rule viewer: %s\n', ME.message);
            end
            
            % Wait a moment before opening next viewer
            pause(1);
            
            % Open rule viewer for Ki
            try
                ruleview(fis_Ki);
                fprintf("Ki rule viewer opened successfully.\n");
            catch ME
                fprintf('Error opening Ki rule viewer: %s\n', ME.message);
            end
            
            % Wait a moment before opening next viewer
            pause(1);
            
            % Open rule viewer for Kd
            try
                ruleview(fis_Kd);
                fprintf("Kd rule viewer opened successfully.\n");
            catch ME
                fprintf('Error opening Kd rule viewer: %s\n', ME.message);
            end
            
            fprintf("All rule viewers opened. Use them to interactively explore the fuzzy rules.\n");
            
        catch ME
            fprintf('Error showing fuzzy rule viewer: %s\n', ME.message);
        end
    end
    
    function [fis_Kp, fis_Ki, fis_Kd] = get_fuzzy_systems()
        % Get the actual fuzzy inference systems from your project's fuzzy_pid_controller
        % This ensures analysis is done on the EXACT systems used in your controller
        
        try
            % Force a dummy call to fuzzy_pid_controller to initialize the persistent systems
            % This will create and store the FIS systems in the persistent variables
            [~, ~, ~] = fuzzy_pid_controller(0, 0);
            
            % Now we need to recreate the systems since we can't access persistent variables directly
            % But we use the EXACT same functions as your controller to ensure identical systems
            
            % Get Kp system - using your project's create_fuzzy_system_Kp function
            fis_Kp = create_fuzzy_system_Kp_for_analysis();
            
            % Get Ki system - using your project's create_fuzzy_system_Ki function  
            fis_Ki = create_fuzzy_system_Ki_for_analysis();
            
            % Get Kd system - using your project's create_fuzzy_system_Kd function
            fis_Kd = create_fuzzy_system_Kd_for_analysis();
            
            fprintf("Fuzzy systems loaded from your project's fuzzy_pid_controller.m\n");
            
        catch ME
            fprintf('Error accessing your project fuzzy systems: %s\n', ME.message);
            % Fallback - this should not happen but provides safety
            fis_Kp = [];
            fis_Ki = [];
            fis_Kd = [];
        end
    end
    
    function fis = create_fuzzy_system_Kp_for_analysis()
        % EXACT copy of create_fuzzy_system_Kp from your fuzzy_pid_controller.m
        % This ensures we analyze the SAME system your controller uses
        
        fis = mamfis('Name', 'KpController');
        
        % Input 1: Error
        fis = addInput(fis, [-1 1], 'Name', 'Error');
        fis = addMF(fis, 'Error', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'Error', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'Error', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'Error', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'Error', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Input 2: Error Rate
        fis = addInput(fis, [-1 1], 'Name', 'ErrorRate');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'ErrorRate', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Output: Kp - tight range 0.07-0.08
        fis = addOutput(fis, [0.07 0.08], 'Name', 'Kp');
        fis = addMF(fis, 'Kp', 'trimf', [0.07 0.07 0.0715], 'Name', 'VS');   % Very Small: 0.070
        fis = addMF(fis, 'Kp', 'trimf', [0.071 0.073 0.0745], 'Name', 'S');  % Small: 0.073
        fis = addMF(fis, 'Kp', 'trimf', [0.0735 0.075 0.0765], 'Name', 'M'); % Medium: 0.075 (optimal)
        fis = addMF(fis, 'Kp', 'trimf', [0.0755 0.077 0.0785], 'Name', 'L');  % Large: 0.077
        fis = addMF(fis, 'Kp', 'trimf', [0.0775 0.08 0.08], 'Name', 'VL');   % Very Large: 0.080
        
        % Optimized rules for Kp around 0.075 - EXACT same as your controller
        rules = [
            1 1 4 1 1;  % If Error is NL and ErrorRate is NL then Kp is L (quick response)
            1 2 4 1 1;  % If Error is NL and ErrorRate is NS then Kp is L
            1 3 3 1 1;  % If Error is NL and ErrorRate is Z then Kp is M
            1 4 2 1 1;  % If Error is NL and ErrorRate is PS then Kp is S
            1 5 1 1 1;  % If Error is NL and ErrorRate is PL then Kp is VS
            2 1 4 1 1;  % If Error is NS and ErrorRate is NL then Kp is L
            2 2 3 1 1;  % If Error is NS and ErrorRate is NS then Kp is M
            2 3 3 1 1;  % If Error is NS and ErrorRate is Z then Kp is M
            2 4 2 1 1;  % If Error is NS and ErrorRate is PS then Kp is S
            2 5 1 1 1;  % If Error is NS and ErrorRate is PL then Kp is VS
            3 1 3 1 1;  % If Error is Z and ErrorRate is NL then Kp is M
            3 2 3 1 1;  % If Error is Z and ErrorRate is NS then Kp is M
            3 3 3 1 1;  % If Error is Z and ErrorRate is Z then Kp is M (optimal)
            3 4 3 1 1;  % If Error is Z and ErrorRate is PS then Kp is M
            3 5 3 1 1;  % If Error is Z and ErrorRate is PL then Kp is M
            4 1 2 1 1;  % If Error is PS and ErrorRate is NL then Kp is S
            4 2 2 1 1;  % If Error is PS and ErrorRate is NS then Kp is S
            4 3 3 1 1;  % If Error is PS and ErrorRate is Z then Kp is M
            4 4 3 1 1;  % If Error is PS and ErrorRate is PS then Kp is M
            4 5 4 1 1;  % If Error is PS and ErrorRate is PL then Kp is L
            5 1 1 1 1;  % If Error is PL and ErrorRate is NL then Kp is VS
            5 2 2 1 1;  % If Error is PL and ErrorRate is NS then Kp is S
            5 3 3 1 1;  % If Error is PL and ErrorRate is Z then Kp is M
            5 4 4 1 1;  % If Error is PL and ErrorRate is PS then Kp is L
            5 5 4 1 1;  % If Error is PL and ErrorRate is PL then Kp is L
        ];
        
        fis = addRule(fis, rules);
    end
    
    function fis = create_fuzzy_system_Ki_for_analysis()
        % EXACT copy of create_fuzzy_system_Ki from your fuzzy_pid_controller.m
        % This ensures we analyze the SAME system your controller uses
        
        fis = mamfis('Name', 'KiController');
        
        % Input 1: Error
        fis = addInput(fis, [-1 1], 'Name', 'Error');
        fis = addMF(fis, 'Error', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'Error', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'Error', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'Error', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'Error', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Input 2: Error Rate
        fis = addInput(fis, [-1 1], 'Name', 'ErrorRate');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'ErrorRate', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Output: Ki - tight range 0.2-0.3
        fis = addOutput(fis, [0.2 0.3], 'Name', 'Ki');
        fis = addMF(fis, 'Ki', 'trimf', [0.2 0.2 0.215], 'Name', 'VS');     % Very Small: 0.20
        fis = addMF(fis, 'Ki', 'trimf', [0.21 0.225 0.24], 'Name', 'S');    % Small: 0.225
        fis = addMF(fis, 'Ki', 'trimf', [0.235 0.25 0.265], 'Name', 'M');   % Medium: 0.25 (optimal)
        fis = addMF(fis, 'Ki', 'trimf', [0.26 0.275 0.29], 'Name', 'L');    % Large: 0.275
        fis = addMF(fis, 'Ki', 'trimf', [0.285 0.3 0.3], 'Name', 'VL');     % Very Large: 0.30
        
        % Optimized rules for Ki around 0.25 - EXACT same as your controller
        rules = [
            1 1 3 1 1;  % If Error is NL and ErrorRate is NL then Ki is M
            1 2 4 1 1;  % If Error is NL and ErrorRate is NS then Ki is L
            1 3 5 1 1;  % If Error is NL and ErrorRate is Z then Ki is VL (eliminate steady-state error)
            1 4 4 1 1;  % If Error is NL and ErrorRate is PS then Ki is L
            1 5 3 1 1;  % If Error is NL and ErrorRate is PL then Ki is M
            2 1 2 1 1;  % If Error is NS and ErrorRate is NL then Ki is S
            2 2 3 1 1;  % If Error is NS and ErrorRate is NS then Ki is M
            2 3 4 1 1;  % If Error is NS and ErrorRate is Z then Ki is L
            2 4 3 1 1;  % If Error is NS and ErrorRate is PS then Ki is M
            2 5 2 1 1;  % If Error is NS and ErrorRate is PL then Ki is S
            3 1 2 1 1;  % If Error is Z and ErrorRate is NL then Ki is S
            3 2 2 1 1;  % If Error is Z and ErrorRate is NS then Ki is S
            3 3 3 1 1;  % If Error is Z and ErrorRate is Z then Ki is M (optimal)
            3 4 2 1 1;  % If Error is Z and ErrorRate is PS then Ki is S
            3 5 2 1 1;  % If Error is Z and ErrorRate is PL then Ki is S
            4 1 2 1 1;  % If Error is PS and ErrorRate is NL then Ki is S
            4 2 3 1 1;  % If Error is PS and ErrorRate is NS then Ki is M
            4 3 4 1 1;  % If Error is PS and ErrorRate is Z then Ki is L
            4 4 3 1 1;  % If Error is PS and ErrorRate is PS then Ki is M
            4 5 2 1 1;  % If Error is PS and ErrorRate is PL then Ki is S
            5 1 3 1 1;  % If Error is PL and ErrorRate is NL then Ki is M
            5 2 4 1 1;  % If Error is PL and ErrorRate is NS then Ki is L
            5 3 5 1 1;  % If Error is PL and ErrorRate is Z then Ki is VL
            5 4 4 1 1;  % If Error is PL and ErrorRate is PS then Ki is L
            5 5 3 1 1;  % If Error is PL and ErrorRate is PL then Ki is M
        ];
        
        fis = addRule(fis, rules);
    end
    
    function fis = create_fuzzy_system_Kd_for_analysis()
        % EXACT copy of create_fuzzy_system_Kd from your fuzzy_pid_controller.m
        % This ensures we analyze the SAME system your controller uses
        
        fis = mamfis('Name', 'KdController');
        
        % Input 1: Error
        fis = addInput(fis, [-1 1], 'Name', 'Error');
        fis = addMF(fis, 'Error', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'Error', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'Error', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'Error', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'Error', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Input 2: Error Rate
        fis = addInput(fis, [-1 1], 'Name', 'ErrorRate');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [-1 -1 -0.6 -0.2], 'Name', 'NL');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.4 -0.2 0], 'Name', 'NS');
        fis = addMF(fis, 'ErrorRate', 'trimf', [-0.1 0 0.1], 'Name', 'Z');
        fis = addMF(fis, 'ErrorRate', 'trimf', [0 0.2 0.4], 'Name', 'PS');
        fis = addMF(fis, 'ErrorRate', 'trapmf', [0.2 0.6 1 1], 'Name', 'PL');
        
        % Output: Kd - tight range 0.0001-0.0005 (very small values for buck converter)
        fis = addOutput(fis, [0.0001 0.0005], 'Name', 'Kd');
        fis = addMF(fis, 'Kd', 'trimf', [0.0001 0.0001 0.00015], 'Name', 'VS');   % Very Small: 0.0001
        fis = addMF(fis, 'Kd', 'trimf', [0.00012 0.00018 0.00024], 'Name', 'S');  % Small: 0.00018
        fis = addMF(fis, 'Kd', 'trimf', [0.00022 0.0003 0.00038], 'Name', 'M');   % Medium: 0.0003
        fis = addMF(fis, 'Kd', 'trimf', [0.00036 0.0004 0.00044], 'Name', 'L');   % Large: 0.0004
        fis = addMF(fis, 'Kd', 'trimf', [0.00042 0.0005 0.0005], 'Name', 'VL');   % Very Large: 0.0005
        
        % Optimized rules for Kd around 0.0001 - EXACT same as your controller
        rules = [
            1 1 4 1 1;  % If Error is NL and ErrorRate is NL then Kd is L (dampen oscillation)
            1 2 3 1 1;  % If Error is NL and ErrorRate is NS then Kd is M
            1 3 2 1 1;  % If Error is NL and ErrorRate is Z then Kd is S
            1 4 1 1 1;  % If Error is NL and ErrorRate is PS then Kd is VS
            1 5 1 1 1;  % If Error is NL and ErrorRate is PL then Kd is VS
            2 1 3 1 1;  % If Error is NS and ErrorRate is NL then Kd is M
            2 2 3 1 1;  % If Error is NS and ErrorRate is NS then Kd is M
            2 3 2 1 1;  % If Error is NS and ErrorRate is Z then Kd is S
            2 4 1 1 1;  % If Error is NS and ErrorRate is PS then Kd is VS
            2 5 1 1 1;  % If Error is NS and ErrorRate is PL then Kd is VS
            3 1 2 1 1;  % If Error is Z and ErrorRate is NL then Kd is S
            3 2 2 1 1;  % If Error is Z and ErrorRate is NS then Kd is S
            3 3 3 1 1;  % If Error is Z and ErrorRate is Z then Kd is M (optimal)
            3 4 2 1 1;  % If Error is Z and ErrorRate is PS then Kd is S
            3 5 2 1 1;  % If Error is Z and ErrorRate is PL then Kd is S
            4 1 2 1 1;  % If Error is PS and ErrorRate is NL then Kd is S
            4 2 2 1 1;  % If Error is PS and ErrorRate is NS then Kd is S
            4 3 2 1 1;  % If Error is PS and ErrorRate is Z then Kd is S
            4 4 3 1 1;  % If Error is PS and ErrorRate is PS then Kd is M
            4 5 3 1 1;  % If Error is PS and ErrorRate is PL then Kd is M
            5 1 1 1 1;  % If Error is PL and ErrorRate is NL then Kd is VS
            5 2 1 1 1;  % If Error is PL and ErrorRate is NS then Kd is VS
            5 3 2 1 1;  % If Error is PL and ErrorRate is Z then Kd is S
            5 4 3 1 1;  % If Error is PL and ErrorRate is PS then Kd is M
            5 5 4 1 1;  % If Error is PL and ErrorRate is PL then Kd is L
        ];
        
        fis = addRule(fis, rules);
    end
    
    function rule_matrix = get_rule_matrix(fis)
        % Extract rule matrix from fuzzy inference system
        rules = fis.Rules;
        rule_matrix = zeros(5, 5);
        
        for i = 1:length(rules)
            error_idx = rules(i).Antecedent(1);
            error_rate_idx = rules(i).Antecedent(2);
            output_idx = rules(i).Consequent(1);
            
            rule_matrix(error_idx, error_rate_idx) = output_idx;
        end
    end
    
    function display_rule_table_plot(rule_matrix, error_labels, error_rate_labels, output_labels, title_str)
        % Display rule table as a heatmap with labels
        
        % Create colormap for the rules
        colormap_values = [
            0.2 0.2 1.0;    % VS - Blue
            0.4 0.4 1.0;    % S  - Light Blue
            0.6 0.6 0.6;    % M  - Gray
            1.0 0.6 0.4;    % L  - Orange
            1.0 0.2 0.2;    % VL - Red
        ];
        
        % Create the plot
        imagesc(rule_matrix);
        colormap(colormap_values);
        
        % Set axis labels
        set(gca, 'XTick', 1:5, 'XTickLabel', error_rate_labels);
        set(gca, 'YTick', 1:5, 'YTickLabel', error_labels);
        xlabel('Error Rate');
        ylabel('Error');
        title(title_str);
        
        % Add text labels for each cell
        for i = 1:5
            for j = 1:5
                text(j, i, output_labels{rule_matrix(i,j)}, ...
                     'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'white');
            end
        end
        
        % Add colorbar
        cbar = colorbar;
        cbar.Ticks = 1:5;
        cbar.TickLabels = output_labels;
        cbar.Label.String = 'Output Level';
    end
    
    function print_rule_table_console(rule_matrix, error_labels, error_rate_labels, output_labels)
        % Print rule table to console in formatted way
        
        fprintf('        ');
        for j = 1:5
            fprintf('%8s', error_rate_labels{j});
        end
        fprintf('\n');
        
        for i = 1:5
            fprintf('%8s', error_labels{i});
            for j = 1:5
                fprintf('%8s', output_labels{rule_matrix(i,j)});
            end
            fprintf('\n');
        end
    end
    
    function arrow(start_point, end_point, varargin)
        % Simple arrow drawing function
        % This is a basic implementation - you might want to use a more sophisticated version
        
        % Parse input arguments
        p = inputParser;
        addParameter(p, 'LineWidth', 1, @isnumeric);
        addParameter(p, 'Color', 'black', @(x) ischar(x) || isnumeric(x));
        parse(p, varargin{:});
        
        % Draw line
        line([start_point(1), end_point(1)], [start_point(2), end_point(2)], ...
             'LineWidth', p.Results.LineWidth, 'Color', p.Results.Color);
        
        % Calculate arrow head
        dx = end_point(1) - start_point(1);
        dy = end_point(2) - start_point(2);
        length_arrow = sqrt(dx^2 + dy^2);
        
        if length_arrow > 0
            % Normalize direction
            dx = dx / length_arrow;
            dy = dy / length_arrow;
            
            % Arrow head parameters
            head_length = 0.1;
            head_angle = 0.3;
            
            % Calculate arrow head points
            head_x1 = end_point(1) - head_length * (dx * cos(head_angle) + dy * sin(head_angle));
            head_y1 = end_point(2) - head_length * (dy * cos(head_angle) - dx * sin(head_angle));
            
            head_x2 = end_point(1) - head_length * (dx * cos(head_angle) - dy * sin(head_angle));
            head_y2 = end_point(2) - head_length * (dy * cos(head_angle) + dx * sin(head_angle));
            
            % Draw arrow head
            line([end_point(1), head_x1], [end_point(2), head_y1], ...
                 'LineWidth', p.Results.LineWidth, 'Color', p.Results.Color);
            line([end_point(1), head_x2], [end_point(2), head_y2], ...
                 'LineWidth', p.Results.LineWidth, 'Color', p.Results.Color);
        end
    end
    
    function clear_data_callback(~, ~)
        % Clear all data arrays
        gui_data.time_data = [];
        gui_data.voltage_data = [];
        gui_data.error_data = [];
        gui_data.setpoint_data = [];
        gui_data.duty_data = [];
        gui_data.Kp_data = [];
        gui_data.Ki_data = [];
        gui_data.Kd_data = [];
        
        % Clear plots
        cla(gui_data.voltage_axes);
        cla(gui_data.error_axes);
        
        % Reset axis properties
        hold(gui_data.voltage_axes, 'on');
        grid(gui_data.voltage_axes, 'on');
        xlabel(gui_data.voltage_axes, 'Time (s)');
        ylabel(gui_data.voltage_axes, 'Voltage (V)');
        title(gui_data.voltage_axes, 'Output Voltage vs Setpoint');
        
        hold(gui_data.error_axes, 'on');
        grid(gui_data.error_axes, 'on');
        xlabel(gui_data.error_axes, 'Time (s)');
        ylabel(gui_data.error_axes, 'Error (V)');
        title(gui_data.error_axes, 'Control Error and PID Parameters');
    end
    
    function control_loop_callback(~, ~)
        % Add counter for GUI updates (update GUI less frequently than control)
        persistent gui_update_counter;
        if isempty(gui_update_counter)
            gui_update_counter = 0;
        end
        
        try
            % Check if we should still be running
            if ~gui_data.is_running
                return;
            end
            
            % Read ADC value from STM32
            adc_value = read_adc_value(gui_data.serial_obj);
            
            if adc_value < 0
                % Failed to read ADC, allow GUI to breathe
                drawnow limitrate;
                return;
            end
            
            % Convert ADC to voltage
            gui_data.current_voltage = adc_to_voltage(adc_value, gui_data.R1, gui_data.R2, gui_data.Vref);
            
            % Calculate current error
            current_error = gui_data.setpoint - gui_data.current_voltage;
            
            % Deadband control: Don't change duty if error is small
            if abs(current_error) < gui_data.deadband_threshold
                % Error is small, keep current duty cycle
                duty_cycle = gui_data.current_duty;
                success = 1; % Don't send command to STM32, just mark as successful
                fprintf('Deadband active - Error: %.3fV (< %.3fV), Duty unchanged: %.1f%%\n', ...
                       current_error, gui_data.deadband_threshold, duty_cycle * 100);
            else
                % Error is significant, calculate new PID control
                [duty_cycle, gui_data.error_integral, gui_data.error_prev] = ...
                    pid_controller(gui_data.setpoint, gui_data.current_voltage, ...
                                  gui_data.error_integral, gui_data.error_prev, ...
                                  gui_data.sampling_time, gui_data.use_fuzzy, ...
                                  gui_data.Kp_manual, gui_data.Ki_manual, gui_data.Kd_manual);
                
                % Send duty cycle to STM32
                success = send_duty_cycle(gui_data.serial_obj, duty_cycle);
                
                % if success
                %     fprintf('PID active - Error: %.3fV, New Duty: %.1f%%\n', ...
                %            current_error, duty_cycle * 100);
                % end
            end
            
            if success
                gui_data.current_duty = duty_cycle;
                gui_data.current_error = current_error;
                
                % Get current PID parameters
                if gui_data.use_fuzzy
                    error_rate = (gui_data.current_error - gui_data.error_prev) / gui_data.sampling_time;
                    [gui_data.current_Kp, gui_data.current_Ki, gui_data.current_Kd] = ...
                        fuzzy_pid_controller(gui_data.current_error, error_rate);
                else
                    gui_data.current_Kp = gui_data.Kp_manual;
                    gui_data.current_Ki = gui_data.Ki_manual;
                    gui_data.current_Kd = gui_data.Kd_manual;
                end
                
                % Log data
                log_data();
                
                % Update GUI less frequently to maintain responsiveness
                gui_update_counter = gui_update_counter + 1;
                if gui_update_counter >= 3  % Update GUI every 3rd control cycle
                    update_monitoring_display();
                    gui_update_counter = 0;
                    
                    % Allow GUI events to be processed
                    drawnow limitrate;
                end
                
                % Update graphs even less frequently (every 10th cycle)
                persistent graph_update_counter;
                if isempty(graph_update_counter)
                    graph_update_counter = 0;
                end
                graph_update_counter = graph_update_counter + 1;
                if graph_update_counter >= 10
                    update_graphs();
                    graph_update_counter = 0;
                end
            else
                % Communication failed, allow GUI to breathe
                drawnow limitrate;
            end
            
        catch ME
            fprintf('Control loop error: %s\n', ME.message);
            % Always allow GUI to breathe after an error
            drawnow limitrate;
        end
    end
    
    function log_data()
        % Add current time
        if isempty(gui_data.time_data)
            current_time = 0;
        else
            current_time = gui_data.time_data(end) + gui_data.sampling_time;
        end
        
        % Append new data
        gui_data.time_data(end+1) = current_time;
        gui_data.voltage_data(end+1) = gui_data.current_voltage;
        gui_data.error_data(end+1) = gui_data.current_error;
        gui_data.setpoint_data(end+1) = gui_data.setpoint;
        gui_data.duty_data(end+1) = gui_data.current_duty * 100; % Convert to percentage
        gui_data.Kp_data(end+1) = gui_data.current_Kp;
        gui_data.Ki_data(end+1) = gui_data.current_Ki;
        gui_data.Kd_data(end+1) = gui_data.current_Kd;
        
        % Limit data length
        if length(gui_data.time_data) > gui_data.max_data_points
            gui_data.time_data = gui_data.time_data(end-gui_data.max_data_points+1:end);
            gui_data.voltage_data = gui_data.voltage_data(end-gui_data.max_data_points+1:end);
            gui_data.error_data = gui_data.error_data(end-gui_data.max_data_points+1:end);
            gui_data.setpoint_data = gui_data.setpoint_data(end-gui_data.max_data_points+1:end);
            gui_data.duty_data = gui_data.duty_data(end-gui_data.max_data_points+1:end);
            gui_data.Kp_data = gui_data.Kp_data(end-gui_data.max_data_points+1:end);
            gui_data.Ki_data = gui_data.Ki_data(end-gui_data.max_data_points+1:end);
            gui_data.Kd_data = gui_data.Kd_data(end-gui_data.max_data_points+1:end);
        end
    end
    
    function update_monitoring_display()
        % Update text displays
        set(gui_data.voltage_text, 'String', sprintf('Output Voltage: %.2f V', gui_data.current_voltage));
        
        % Show deadband status in error display
        if abs(gui_data.current_error) < gui_data.deadband_threshold
            set(gui_data.error_text, 'String', sprintf('Error: %.3f V (DEADBAND)', gui_data.current_error), ...
                'ForegroundColor', [0.8, 0.6, 0.2]); % Orange color for deadband
        else
            set(gui_data.error_text, 'String', sprintf('Error: %.3f V (ACTIVE)', gui_data.current_error), ...
                'ForegroundColor', [0, 0, 0]); % Black color for active control
        end
        
        set(gui_data.duty_text, 'String', sprintf('Duty Cycle: %.1f%%', gui_data.current_duty * 100));
        set(gui_data.pid_text, 'String', sprintf('PID: Kp=%.3f Ki=%.2f Kd=%.5f', ...
            gui_data.current_Kp, gui_data.current_Ki, gui_data.current_Kd));
    end
    
    function update_graphs()
        if isempty(gui_data.time_data)
            return;
        end
        
        % Store handle to voltage plot lines for faster updates
        persistent voltage_line setpoint_line error_line Kp_line Ki_line Kd_line;
        
        try
            % Update voltage graph - use handle-based updates for better performance
            if isempty(voltage_line) || ~isvalid(voltage_line)
                % First time or invalid handle - create new plots
                cla(gui_data.voltage_axes);
                hold(gui_data.voltage_axes, 'on');
                voltage_line = plot(gui_data.voltage_axes, gui_data.time_data, gui_data.voltage_data, 'b-', 'LineWidth', 2);
                setpoint_line = plot(gui_data.voltage_axes, gui_data.time_data, gui_data.setpoint_data, 'r--', 'LineWidth', 1.5);
                legend(gui_data.voltage_axes, {'Output Voltage', 'Setpoint'}, 'Location', 'best');
                grid(gui_data.voltage_axes, 'on');
                xlabel(gui_data.voltage_axes, 'Time (s)');
                ylabel(gui_data.voltage_axes, 'Voltage (V)');
                title(gui_data.voltage_axes, 'Output Voltage vs Setpoint');
            else
                % Update existing plots - much faster than recreating
                set(voltage_line, 'XData', gui_data.time_data, 'YData', gui_data.voltage_data);
                set(setpoint_line, 'XData', gui_data.time_data, 'YData', gui_data.setpoint_data);
                
                % Only update axis limits if necessary
                xlim_current = xlim(gui_data.voltage_axes);
                if length(gui_data.time_data) > 0 && ...
                   (gui_data.time_data(end) > xlim_current(2) || gui_data.time_data(1) < xlim_current(1))
                    xlim(gui_data.voltage_axes, [max(0, gui_data.time_data(end)-50), gui_data.time_data(end)+2]);
                end
            end
            
            % Update error graph with PID parameters
            if isempty(error_line) || ~isvalid(error_line)
                % First time or invalid handle - create new plots
                cla(gui_data.error_axes);
                hold(gui_data.error_axes, 'on');
                
                % Plot error on left y-axis
                yyaxis(gui_data.error_axes, 'left');
                error_line = plot(gui_data.error_axes, gui_data.time_data, gui_data.error_data, 'g-', 'LineWidth', 2);
                ylabel(gui_data.error_axes, 'Error (V)');
                
                % Plot PID parameters on right y-axis
                yyaxis(gui_data.error_axes, 'right');
                Kp_line = plot(gui_data.error_axes, gui_data.time_data, gui_data.Kp_data, 'm-', 'LineWidth', 1);
                Ki_line = plot(gui_data.error_axes, gui_data.time_data, gui_data.Ki_data, 'c-', 'LineWidth', 1);
                Kd_line = plot(gui_data.error_axes, gui_data.time_data, gui_data.Kd_data * 10000, 'y-', 'LineWidth', 1);
                ylabel(gui_data.error_axes, 'PID Parameters');
                
                legend(gui_data.error_axes, {'Error', 'Kp', 'Ki', 'Kd×10000'}, 'Location', 'best');
                grid(gui_data.error_axes, 'on');
                xlabel(gui_data.error_axes, 'Time (s)');
                title(gui_data.error_axes, 'Control Error and PID Parameters');
            else
                % Update existing plots - much faster
                yyaxis(gui_data.error_axes, 'left');
                set(error_line, 'XData', gui_data.time_data, 'YData', gui_data.error_data);
                
                yyaxis(gui_data.error_axes, 'right');
                set(Kp_line, 'XData', gui_data.time_data, 'YData', gui_data.Kp_data);
                set(Ki_line, 'XData', gui_data.time_data, 'YData', gui_data.Ki_data);
                set(Kd_line, 'XData', gui_data.time_data, 'YData', gui_data.Kd_data * 10000);
                
                % Only update axis limits if necessary
                xlim_current = xlim(gui_data.error_axes);
                if length(gui_data.time_data) > 0 && ...
                   (gui_data.time_data(end) > xlim_current(2) || gui_data.time_data(1) < xlim_current(1))
                    xlim(gui_data.error_axes, [max(0, gui_data.time_data(end)-50), gui_data.time_data(end)+2]);
                end
            end
            
        catch ME
            % If plot update fails, reset handles and try again next time
            fprintf('Graph update error: %s\n', ME.message);
            voltage_line = [];
            setpoint_line = [];
            error_line = [];
            Kp_line = [];
            Ki_line = [];
            Kd_line = [];
        end
    end
    
    function update_gui_state()
        if gui_data.is_running
            set(gui_data.start_button, 'Enable', 'off');
            set(gui_data.stop_button, 'Enable', 'on');
            set(gui_data.stop_button, 'String', 'STOP');
            set(gui_data.status_text, 'String', 'Status: Running', ...
                'ForegroundColor', [0.2, 0.8, 0.2]);
        else
            set(gui_data.start_button, 'Enable', 'on');
            set(gui_data.stop_button, 'Enable', 'off');
            set(gui_data.stop_button, 'String', 'STOP');
            set(gui_data.status_text, 'String', 'Status: Stopped', ...
                'ForegroundColor', [0.8, 0.2, 0.2]);
        end
        
        % Enable/disable manual PID controls based on fuzzy mode
        if gui_data.use_fuzzy
            enable_state = 'off';
        else
            enable_state = 'on';
        end
        set(gui_data.Kp_edit, 'Enable', enable_state);
        set(gui_data.Ki_edit, 'Enable', enable_state);
        set(gui_data.Kd_edit, 'Enable', enable_state);
    end
    
    function close_gui(~, ~)
        % Emergency stop if running
        if gui_data.is_running
            fprintf("Window closing - Emergency stop triggered\n");
            stop_callback();
        end
        
        % Force cleanup of any remaining objects
        try
            % Clean up any remaining timers
            all_timers = timerfindall;
            for i = 1:length(all_timers)
                if isvalid(all_timers(i))
                    stop(all_timers(i));
                    delete(all_timers(i));
                end
            end
            
            % Clean up any remaining serial objects
            serial_objects = instrfindall('Type', 'serialport');
            for i = 1:length(serial_objects)
                if isvalid(serial_objects(i))
                    delete(serial_objects(i));
                end
            end
        catch ME
            fprintf('Cleanup error during window close: %s\n', ME.message);
        end
        
        % Close figure
        delete(gui_data.fig);
        fprintf("GUI closed safely\n");
    end
    

    
end 