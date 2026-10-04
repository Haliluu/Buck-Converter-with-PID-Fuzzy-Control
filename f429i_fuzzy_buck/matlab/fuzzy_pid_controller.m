function [Kp, Ki, Kd] = fuzzy_pid_controller(error, error_rate)
    % Adaptive PID Fuzzy Controller for Buck Converter
    % Optimized for Kp=0.075, Ki=0.25, Kd=0.0001 based on manual tuning
    
    persistent fis_Kp fis_Ki fis_Kd
    
    if isempty(fis_Kp)
        % Initialize fuzzy inference systems for Kp, Ki, Kd
        fis_Kp = create_fuzzy_system_Kp();
        fis_Ki = create_fuzzy_system_Ki();
        fis_Kd = create_fuzzy_system_Kd();
    end
    
    % Normalize inputs (error and error_rate should be in range [-1, 1])
    error_norm = normalize_error(error);
    error_rate_norm = normalize_error_rate(error_rate);
    
    % Evaluate fuzzy systems
    Kp = evalfis(fis_Kp, [error_norm, error_rate_norm]);
    Ki = evalfis(fis_Ki, [error_norm, error_rate_norm]);
    Kd = evalfis(fis_Kd, [error_norm, error_rate_norm]);
    
    % Ensure values stay within tight optimized ranges around manual optimal values
    Kp = max(0.07, min(0.08, Kp));    % Tight range: 0.07-0.08 around optimal Kp=0.075
    Ki = max(0.2, min(0.3, Ki));      % Tight range: 0.2-0.3 around optimal Ki=0.25
    Kd = max(0.0001, min(0.0005, Kd)); % Tight range: 0.0001-0.0005 around optimal Kd=0.0001
end

function fis = create_fuzzy_system_Kp()
    % Create fuzzy inference system for Kp tuning - optimized for 0.075
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
    
    % Optimized rules for Kp around 0.075
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

function fis = create_fuzzy_system_Ki()
    % Create fuzzy inference system for Ki tuning - optimized for 0.25
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
    
    % Optimized rules for Ki around 0.25 (Ki should be higher for persistent errors)
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

function fis = create_fuzzy_system_Kd()
    % Create fuzzy inference system for Kd tuning - optimized for 0.0001
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
    
    % Optimized rules for Kd around 0.0001 (low Kd for stability in buck converter)
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

function norm_error = normalize_error(error)
    % Normalize error to [-1, 1] range
    % Assuming maximum expected error is 5V
    max_error = 5;
    norm_error = max(-1, min(1, error / max_error));
end

function norm_error_rate = normalize_error_rate(error_rate)
    % Normalize error rate to [-1, 1] range
    % Assuming maximum expected error rate is 10V/s
    max_error_rate = 10;
    norm_error_rate = max(-1, min(1, error_rate / max_error_rate));
end 