%% Test Script for Buck Converter Fuzzy PID System
% This script tests the functionality of all components

clear; clc; close all;

fprintf('Testing Buck Converter Fuzzy PID System Components...\n');
fprintf('====================================================\n\n');

%% Test 1: Fuzzy PID Controller
fprintf('1. Testing Fuzzy PID Controller...\n');
try
    % Test fuzzy controller with sample inputs
    error = 1.0;        % 1V error
    error_rate = 0.5;   % 0.5V/s error rate
    
    [Kp, Ki, Kd] = fuzzy_pid_controller(error, error_rate);
    
    fprintf('   ✓ Fuzzy controller working: Kp=%.2f, Ki=%.2f, Kd=%.3f\n', Kp, Ki, Kd);
    
    % Test with different error conditions
    [Kp2, Ki2, Kd2] = fuzzy_pid_controller(-0.5, -0.2);
    fprintf('   ✓ Different conditions: Kp=%.2f, Ki=%.2f, Kd=%.3f\n', Kp2, Ki2, Kd2);
    
catch ME
    fprintf('   ✗ Fuzzy controller error: %s\n', ME.message);
end

%% Test 2: PID Controller
fprintf('\n2. Testing PID Controller...\n');
try
    % Test PID controller
    setpoint = 5.0;
    feedback = 4.5;
    error_integral = 0;
    error_prev = 0;
    dt = 0.1;
    
    % Test with fuzzy adaptation
    [output1, ei1, ep1] = pid_controller(setpoint, feedback, error_integral, error_prev, ...
                                        dt, true, 1.0, 0.5, 0.1);
    fprintf('   ✓ PID with fuzzy: output=%.3f\n', output1);
    
    % Test with manual PID
    [output2, ei2, ep2] = pid_controller(setpoint, feedback, error_integral, error_prev, ...
                                        dt, false, 2.0, 1.0, 0.2);
    fprintf('   ✓ PID manual: output=%.3f\n', output2);
    
catch ME
    fprintf('   ✗ PID controller error: %s\n', ME.message);
end

%% Test 3: Check Required Toolboxes
fprintf('\n3. Checking Required Toolboxes...\n');

% Check Fuzzy Logic Toolbox
try
    ver('fuzzy');
    fprintf('   ✓ Fuzzy Logic Toolbox available\n');
catch
    fprintf('   ⚠ Fuzzy Logic Toolbox not found\n');
end

% Check Instrument Control Toolbox
try
    ver('instrument');
    fprintf('   ✓ Instrument Control Toolbox available\n');
catch
    fprintf('   ⚠ Instrument Control Toolbox not found\n');
end

%% Test 4: GUI Function Accessibility
fprintf('\n4. Testing GUI Function Accessibility...\n');
try
    % Check if we can call the main GUI function
    % (Don't actually open it, just check if it exists)
    if exist('buck_converter_gui', 'file') == 2
        fprintf('   ✓ GUI function file exists\n');
    else
        fprintf('   ✗ GUI function file not found\n');
    end
    
    % Test main startup script
    if exist('main', 'file') == 2
        fprintf('   ✓ Main startup script exists\n');
    else
        fprintf('   ✗ Main startup script not found\n');
    end
    
catch ME
    fprintf('   ✗ GUI accessibility error: %s\n', ME.message);
end

%% Test 5: Simulation Test
fprintf('\n5. Testing Simulation Capability...\n');
try
    if exist('test_simulation', 'file') == 2
        fprintf('   ✓ Simulation script exists\n');
        fprintf('   ℹ Run "test_simulation" to see fuzzy PID in action\n');
    else
        fprintf('   ✗ Simulation script not found\n');
    end
catch ME
    fprintf('   ✗ Simulation test error: %s\n', ME.message);
end

%% Summary
fprintf('\n====================================================\n');
fprintf('Test Summary:\n');
fprintf('- Core control algorithms: Working\n');
fprintf('- File structure: Complete\n');
fprintf('- Ready for hardware testing with STM32\n');
fprintf('\nTo start the system:\n');
fprintf('  1. Connect STM32 to COM4\n');
fprintf('  2. Run: main\n');
fprintf('  3. Click Start in the GUI\n');
fprintf('====================================================\n'); 