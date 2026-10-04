function voltage = trial(adc_value, R1, R2, Vref)
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

trial()