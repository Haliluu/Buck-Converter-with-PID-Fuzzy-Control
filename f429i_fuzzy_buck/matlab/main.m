%% Buck Converter Fuzzy PID Control System
% Main startup script for the Adaptive PID Fuzzy Control for Buck Converter
% 
% This system provides:
% - Real-time fuzzy adaptation of PID parameters
% - MATLAB GUI interface with live monitoring
% - Serial communication with STM32 (COM4)
% - Voltage and error graphs
% - Configurable voltage divider settings
%
% Created for STM32F429 Buck Converter Project

%% Clear workspace and command window
clear; clc; close all;

%% Add current directory to path
addpath(pwd);

%% Display system information
fprintf('========================================\n');
fprintf('Buck Converter Fuzzy PID Control System\n');
fprintf('========================================\n');
fprintf('Features:\n');
fprintf('  • Adaptive PID with Fuzzy Logic\n');
fprintf('  • Real-time monitoring GUI\n');
fprintf('  • Serial communication (COM4)\n');
fprintf('  • Live voltage/error graphs\n');
fprintf('  • Configurable parameters\n');
fprintf('========================================\n\n');

%% Check required toolboxes
fprintf('Checking required toolboxes...\n');

% Check Fuzzy Logic Toolbox
try
    ver('fuzzy');
    fprintf('✓ Fuzzy Logic Toolbox found\n');
catch
    warning('⚠ Fuzzy Logic Toolbox not found. Please install it to use fuzzy PID adaptation.');
end

% Check Instrument Control Toolbox (for serial communication)
try
    ver('instrument');
    fprintf('✓ Instrument Control Toolbox found\n');
catch
    warning('⚠ Instrument Control Toolbox not found. Please install it for serial communication.');
end

fprintf('\n');

%% System Configuration
fprintf('System Configuration:\n');
fprintf('  Default Setpoint: 5.0V\n');
fprintf('  Voltage Divider: R1=1kΩ, R2=1kΩ\n');
fprintf('  Serial Port: COM4 (115200 baud)\n');
fprintf('  Sampling Time: 0.1s\n');
fprintf('  Control Mode: Fuzzy PID (default)\n');
fprintf('\n');

%% Launch GUI
fprintf('Launching Buck Converter Control GUI...\n');
try
    buck_converter_gui();
    fprintf('✓ GUI launched successfully\n');
    fprintf('\nInstructions:\n');
    fprintf('1. Connect STM32 to COM4 port\n');
    fprintf('2. Set desired output voltage setpoint\n');
    fprintf('3. Configure voltage divider values if different\n');
    fprintf('4. Click "Start" to begin control\n');
    fprintf('5. Monitor real-time graphs and PID parameters\n');
    fprintf('6. Use "Stop" to halt control safely\n');
    fprintf('\nNote: The system will automatically adapt PID parameters\n');
    fprintf('      based on error and error rate using fuzzy logic.\n');
    
catch ME
    fprintf('✗ Error launching GUI: %s\n', ME.message);
    fprintf('Please check that all required files are in the current directory.\n');
end

fprintf('\n========================================\n'); 