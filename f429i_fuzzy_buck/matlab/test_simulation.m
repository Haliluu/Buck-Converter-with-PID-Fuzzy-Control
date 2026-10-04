%% Buck Converter Fuzzy PID Control - Simulation Mode
% This script allows testing of the fuzzy PID controller without hardware
% It simulates a buck converter response and demonstrates the adaptive control

clear; clc; close all;

%% Simulation Parameters
sim_time = 30;              % Simulation time (seconds)
dt = 0.1;                   % Time step (seconds)
time = 0:dt:sim_time;       % Time vector
n_steps = length(time);

%% Buck Converter Model Parameters
% Simplified buck converter transfer function: Vout = Duty * Vin * H(s)
Vin = 12;                   % Input voltage (V)
L = 100e-6;                 % Inductance (H)
C = 470e-6;                 % Capacitance (F)
R_load = 10;                % Load resistance (Ohm)

% Transfer function from duty cycle to output voltage
% H(s) = Vin / (s^2*L*C + s*L/R + 1)
wn = 1/sqrt(L*C);           % Natural frequency
zeta = 1/(2*R_load) * sqrt(L/C);  % Damping ratio

fprintf('Buck Converter Model:\n');
fprintf('  Input Voltage: %.1f V\n', Vin);
fprintf('  Natural Frequency: %.1f rad/s\n', wn);
fprintf('  Damping Ratio: %.3f\n', zeta);
fprintf('  Load Resistance: %.1f Ω\n', R_load);
fprintf('\n');

%% Initialize Variables
setpoint = 5.0;             % Desired output voltage
voltage_output = zeros(1, n_steps);
duty_cycle = zeros(1, n_steps);
error_signal = zeros(1, n_steps);
Kp_values = zeros(1, n_steps);
Ki_values = zeros(1, n_steps);
Kd_values = zeros(1, n_steps);

% PID controller states
error_integral = 0;
error_prev = 0;

% Buck converter states (2nd order system)
x1 = 0;     % Capacitor voltage
x2 = 0;     % Inductor current derivative

% Test scenarios
setpoint_changes = [0, 5.0; 10, 8.0; 20, 3.0];  % [time, setpoint]
load_changes = [0, R_load; 15, R_load*0.5];     % [time, load]

fprintf('Starting Simulation...\n');
fprintf('Test Scenarios:\n');
fprintf('  t=0s:  Setpoint = 5.0V\n');
fprintf('  t=10s: Setpoint = 8.0V\n');
fprintf('  t=15s: Load change (resistance halved)\n');
fprintf('  t=20s: Setpoint = 3.0V\n');
fprintf('\n');

%% Simulation Loop
for i = 1:n_steps
    current_time = time(i);
    
    % Update setpoint based on test scenario
    for j = 1:size(setpoint_changes, 1)
        if current_time >= setpoint_changes(j, 1)
            setpoint = setpoint_changes(j, 2);
        end
    end
    
    % Update load resistance based on test scenario
    current_load = R_load;
    for j = 1:size(load_changes, 1)
        if current_time >= load_changes(j, 1)
            current_load = load_changes(j, 2);
        end
    end
    
    % Update damping ratio with new load
    zeta = 1/(2*current_load) * sqrt(L/C);
    
    % Current output voltage (from buck converter model)
    current_voltage = voltage_output(i) + x1;
    
    % Add some noise to simulate real measurements
    noise = 0.02 * randn(); % 20mV RMS noise
    measured_voltage = current_voltage + noise;
    
    % Calculate error
    error = setpoint - measured_voltage;
    error_signal(i) = error;
    
    % Get fuzzy PID parameters
    error_rate = (error - error_prev) / dt;
    [Kp, Ki, Kd] = fuzzy_pid_controller(error, error_rate);
    
    % Store PID parameters
    Kp_values(i) = Kp;
    Ki_values(i) = Ki;
    Kd_values(i) = Kd;
    
    % Calculate PID control output
    [duty, error_integral, error_prev] = pid_controller(setpoint, measured_voltage, ...
        error_integral, error_prev, dt, true, 1.0, 0.5, 0.1);
    
    duty_cycle(i) = duty;
    
    % Buck converter dynamics (simplified 2nd order model)
    % State equations:
    % dx1/dt = x2
    % dx2/dt = -wn^2*x1 - 2*zeta*wn*x2 + wn^2*Vin*duty
    
    if i > 1
        % Runge-Kutta 4th order integration
        k1_x1 = x2;
        k1_x2 = -wn^2*x1 - 2*zeta*wn*x2 + wn^2*Vin*duty;
        
        k2_x1 = x2 + dt/2*k1_x2;
        k2_x2 = -wn^2*(x1 + dt/2*k1_x1) - 2*zeta*wn*(x2 + dt/2*k1_x2) + wn^2*Vin*duty;
        
        k3_x1 = x2 + dt/2*k2_x2;
        k3_x2 = -wn^2*(x1 + dt/2*k2_x1) - 2*zeta*wn*(x2 + dt/2*k2_x2) + wn^2*Vin*duty;
        
        k4_x1 = x2 + dt*k3_x2;
        k4_x2 = -wn^2*(x1 + dt*k3_x1) - 2*zeta*wn*(x2 + dt*k3_x2) + wn^2*Vin*duty;
        
        x1 = x1 + dt/6*(k1_x1 + 2*k2_x1 + 2*k3_x1 + k4_x1);
        x2 = x2 + dt/6*(k1_x2 + 2*k2_x2 + 2*k3_x2 + k4_x2);
    end
    
    voltage_output(i) = x1;
    
    % Display progress
    if mod(i, 50) == 0 || i == n_steps
        fprintf('Time: %4.1fs, Setpoint: %.1fV, Output: %.2fV, Error: %.3fV, Duty: %.1f%%\n', ...
            current_time, setpoint, current_voltage, error, duty*100);
    end
end

fprintf('\nSimulation Complete!\n\n');

%% Results Analysis
fprintf('Performance Analysis:\n');

% Calculate performance metrics
settling_time_threshold = 0.02; % 2% settling
steady_state_error = abs(error_signal(end));
max_overshoot = max(voltage_output) - setpoint;
rise_time_idx = find(voltage_output >= 0.9*setpoint, 1);
rise_time = time(rise_time_idx) * 1000; % Convert to ms

fprintf('  Steady-state Error: %.3f V\n', steady_state_error);
fprintf('  Maximum Overshoot: %.3f V\n', max_overshoot);
fprintf('  Rise Time: %.1f ms\n', rise_time);
fprintf('  Final Duty Cycle: %.1f%%\n', duty_cycle(end)*100);
fprintf('  Final PID: Kp=%.2f, Ki=%.2f, Kd=%.3f\n', ...
    Kp_values(end), Ki_values(end), Kd_values(end));

%% Plot Results
figure('Name', 'Buck Converter Fuzzy PID Simulation Results', 'Position', [100, 100, 1200, 800]);

% Voltage response
subplot(3, 2, 1);
plot(time, voltage_output, 'b-', 'LineWidth', 2);
hold on;
% Plot setpoint changes
current_setpoint = setpoint_changes(1, 2);
setpoint_vector = ones(1, n_steps) * current_setpoint;
for i = 1:n_steps
    for j = 1:size(setpoint_changes, 1)
        if time(i) >= setpoint_changes(j, 1)
            setpoint_vector(i) = setpoint_changes(j, 2);
        end
    end
end
plot(time, setpoint_vector, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Output Voltage Response');
legend('Output Voltage', 'Setpoint', 'Location', 'best');

% Error signal
subplot(3, 2, 2);
plot(time, error_signal, 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Error (V)');
title('Control Error');

% Duty cycle
subplot(3, 2, 3);
plot(time, duty_cycle*100, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Duty Cycle (%)');
title('PWM Duty Cycle');

% PID parameters
subplot(3, 2, 4);
plot(time, Kp_values, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, Ki_values, 'g-', 'LineWidth', 1.5);
plot(time, Kd_values*10, 'b-', 'LineWidth', 1.5); % Scale Kd for visibility
grid on;
xlabel('Time (s)');
ylabel('PID Parameters');
title('Adaptive PID Parameters');
legend('Kp', 'Ki', 'Kd×10', 'Location', 'best');

% Phase portrait (error vs error rate)
subplot(3, 2, 5);
error_rate = [0, diff(error_signal)/dt];
plot(error_signal, error_rate, 'c-', 'LineWidth', 1);
grid on;
xlabel('Error (V)');
ylabel('Error Rate (V/s)');
title('Phase Portrait (Error vs Error Rate)');

% Performance metrics over time
subplot(3, 2, 6);
iae = cumsum(abs(error_signal)) * dt; % Integral Absolute Error
ise = cumsum(error_signal.^2) * dt;   % Integral Square Error
plot(time, iae, 'k-', 'LineWidth', 2);
hold on;
plot(time, ise, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Performance Index');
title('Performance Metrics');
legend('IAE', 'ISE', 'Location', 'best');

%% Compare with Manual PID
fprintf('\nComparing with Manual PID...\n');

% Reset variables for manual PID simulation
voltage_output_manual = zeros(1, n_steps);
error_integral_manual = 0;
error_prev_manual = 0;
x1_manual = 0;
x2_manual = 0;

% Manual PID parameters (typical values)
Kp_manual = 2.0;
Ki_manual = 1.0;
Kd_manual = 0.1;

for i = 1:n_steps
    current_time = time(i);
    
    % Update setpoint
    setpoint_manual = setpoint_changes(1, 2);
    for j = 1:size(setpoint_changes, 1)
        if current_time >= setpoint_changes(j, 1)
            setpoint_manual = setpoint_changes(j, 2);
        end
    end
    
    % Current output voltage
    current_voltage_manual = voltage_output_manual(i) + x1_manual;
    measured_voltage_manual = current_voltage_manual + 0.02 * randn();
    
    % Manual PID control
    [duty_manual, error_integral_manual, error_prev_manual] = ...
        pid_controller(setpoint_manual, measured_voltage_manual, ...
        error_integral_manual, error_prev_manual, dt, false, ...
        Kp_manual, Ki_manual, Kd_manual);
    
    % Buck converter dynamics (same as before)
    if i > 1
        k1_x1 = x2_manual;
        k1_x2 = -wn^2*x1_manual - 2*zeta*wn*x2_manual + wn^2*Vin*duty_manual;
        
        k2_x1 = x2_manual + dt/2*k1_x2;
        k2_x2 = -wn^2*(x1_manual + dt/2*k1_x1) - 2*zeta*wn*(x2_manual + dt/2*k1_x2) + wn^2*Vin*duty_manual;
        
        k3_x1 = x2_manual + dt/2*k2_x2;
        k3_x2 = -wn^2*(x1_manual + dt/2*k2_x1) - 2*zeta*wn*(x2_manual + dt/2*k2_x2) + wn^2*Vin*duty_manual;
        
        k4_x1 = x2_manual + dt*k3_x2;
        k4_x2 = -wn^2*(x1_manual + dt*k3_x1) - 2*zeta*wn*(x2_manual + dt*k3_x2) + wn^2*Vin*duty_manual;
        
        x1_manual = x1_manual + dt/6*(k1_x1 + 2*k2_x1 + 2*k3_x1 + k4_x1);
        x2_manual = x2_manual + dt/6*(k1_x2 + 2*k2_x2 + 2*k3_x2 + k4_x2);
    end
    
    voltage_output_manual(i) = x1_manual;
end

% Plot comparison
figure('Name', 'Fuzzy vs Manual PID Comparison', 'Position', [150, 150, 1000, 600]);

subplot(2, 1, 1);
plot(time, voltage_output, 'b-', 'LineWidth', 2);
hold on;
plot(time, voltage_output_manual, 'r-', 'LineWidth', 2);
plot(time, setpoint_vector, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Voltage Response Comparison');
legend('Fuzzy PID', 'Manual PID', 'Setpoint', 'Location', 'best');

subplot(2, 1, 2);
error_fuzzy = setpoint_vector - voltage_output;
error_manual = setpoint_vector - voltage_output_manual;
plot(time, abs(error_fuzzy), 'b-', 'LineWidth', 2);
hold on;
plot(time, abs(error_manual), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Absolute Error (V)');
title('Error Comparison');
legend('Fuzzy PID', 'Manual PID', 'Location', 'best');

% Performance comparison
iae_fuzzy = sum(abs(error_fuzzy)) * dt;
iae_manual = sum(abs(error_manual)) * dt;
ise_fuzzy = sum(error_fuzzy.^2) * dt;
ise_manual = sum(error_manual.^2) * dt;

fprintf('Performance Comparison:\n');
fprintf('  Fuzzy PID - IAE: %.3f, ISE: %.3f\n', iae_fuzzy, ise_fuzzy);
fprintf('  Manual PID - IAE: %.3f, ISE: %.3f\n', iae_manual, ise_manual);
fprintf('  Improvement - IAE: %.1f%%, ISE: %.1f%%\n', ...
    (iae_manual - iae_fuzzy)/iae_manual*100, ...
    (ise_manual - ise_fuzzy)/ise_manual*100);

fprintf('\nSimulation demonstrates the adaptive behavior of fuzzy PID control!\n'); 