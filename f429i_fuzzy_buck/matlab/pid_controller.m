function [control_output, error_integral, error_prev] = pid_controller(setpoint, feedback, ...
    error_integral, error_prev, dt, use_fuzzy, Kp_manual, Ki_manual, Kd_manual)
    % PID Controller for Buck Converter with Fuzzy Adaptation
    % Inputs:
    %   setpoint - desired output voltage
    %   feedback - actual output voltage from ADC
    %   error_integral - accumulated error for integral term
    %   error_prev - previous error for derivative term
    %   dt - sampling time
    %   use_fuzzy - flag to use fuzzy adaptation (1) or manual PID (0)
    %   Kp_manual, Ki_manual, Kd_manual - manual PID parameters
    % Outputs:
    %   control_output - duty cycle (0-1)
    %   error_integral - updated integral error
    %   error_prev - current error for next iteration
    
    % Calculate error
    error = setpoint - feedback;
    
    % Calculate error rate (derivative of error)
    error_rate = (error - error_prev) / dt;
    
    % Get PID parameters
    if use_fuzzy
        % Use fuzzy logic to adapt PID parameters
        [Kp, Ki, Kd] = fuzzy_pid_controller(error, error_rate);
    else
        % Use manual PID parameters
        Kp = Kp_manual;
        Ki = Ki_manual;
        Kd = Kd_manual;
    end
    
    % Calculate PID terms
    P_term = Kp * error;
    
    % Integral term with windup protection
    error_integral = error_integral + error * dt;
    % % Anti-windup: limit integral term
    % max_integral = 1.0; % maximum duty cycle
    % if Ki > 0
    %     integral_limit = max_integral / Ki;
    %     error_integral = max(-integral_limit, min(integral_limit, error_integral));
    % end
    I_term = Ki * error_integral;
    
    D_term = Kd * error_rate;
    
    % Calculate total control output
    control_output = P_term + I_term + D_term;
    
    % Saturate output to valid duty cycle range [0, 1]
    control_output = max(0, min(1, control_output));
    
    % Additional anti-windup: reset integral if output is saturated
    if (control_output >= 1 && error > 0) || (control_output <= 0 && error < 0)
        error_integral = error_integral - error * dt;
    end
    
    % Update previous error
    error_prev = error;
end 