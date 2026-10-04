# Buck Converter Adaptive PID Fuzzy Control System

This MATLAB project implements an adaptive PID controller with fuzzy logic for controlling a buck converter's output voltage. The system provides real-time monitoring, parameter tuning, and communication with STM32 microcontroller.

## Features

- **Adaptive PID Control**: Fuzzy logic automatically adjusts Kp, Ki, and Kd parameters based on error and error rate
- **Real-time GUI**: Live monitoring with voltage graphs, error plots, and PID parameter display
- **Serial Communication**: Bidirectional communication with STM32 via COM4 port
- **Voltage Divider Support**: Configurable voltage divider calculation for accurate voltage measurement
- **Manual/Auto Mode**: Switch between fuzzy adaptive control and manual PID parameters
- **Data Logging**: Real-time data collection and visualization

## Files Description

### Core Files
- `main.m` - Main startup script that launches the entire system
- `buck_converter_gui.m` - Main GUI interface with real-time monitoring, control, and integrated serial communication
- `fuzzy_pid_controller.m` - Fuzzy logic system for adaptive PID parameter tuning
- `pid_controller.m` - PID controller implementation with anti-windup protection
- `test_simulation.m` - Simulation script for testing without hardware

### Configuration
- `README.md` - This documentation file

## System Requirements

### MATLAB Toolboxes
- **Fuzzy Logic Toolbox** - Required for fuzzy PID adaptation
- **Instrument Control Toolbox** - Required for serial communication

### Hardware
- STM32F429 microcontroller with buck converter circuit
- USB to Serial connection (COM4)
- Voltage divider network for voltage sensing

## Quick Start

1. **Install Required Toolboxes**
   ```matlab
   % Check if toolboxes are installed
   ver('fuzzy')
   ver('instrument')
   ```

2. **Connect Hardware**
   - Connect STM32 to PC via USB (should appear as COM4)
   - Ensure buck converter circuit is properly connected
   - Verify voltage divider is connected to ADC input

3. **Launch System**
   ```matlab
   % Run main script
   main
   ```

4. **Configure Parameters**
   - Set desired output voltage (setpoint)
   - Configure voltage divider values (R1, R2)
   - Choose between Fuzzy PID or Manual PID mode

5. **Start Control**
   - Click "Start" button to begin control loop
   - Monitor real-time voltage and error graphs
   - Observe automatic PID parameter adaptation

## System Architecture

### Fuzzy Logic Controller
The fuzzy controller uses two inputs:
- **Error**: Difference between setpoint and actual voltage
- **Error Rate**: Rate of change of error

And produces three outputs:
- **Kp**: Proportional gain
- **Ki**: Integral gain  
- **Kd**: Derivative gain

### Membership Functions
- **Inputs**: 5 fuzzy sets each (NL, NS, Z, PS, PL)
- **Outputs**: 5 fuzzy sets each (S, MS, M, ML, L)
- **Rules**: 25 rules per parameter (75 total rules)

### Communication Protocol
The system communicates with STM32 using simple text commands:

**To STM32:**
```
DUTY:XX\n    % Send duty cycle (0-100%)
ADC?\n       % Request ADC reading
```

**From STM32:**
```
ADC:XXXX\n   % ADC value (0-4095)
```

## Voltage Divider Calculation

The system accounts for voltage divider networks:
```
Vout = Vadc × (R1 + R2) / R2
```

Where:
- `Vadc` = Voltage measured by ADC
- `R1` = Upper resistor (connected to buck converter output)
- `R2` = Lower resistor (connected to ground)
- `Vout` = Actual buck converter output voltage

## GUI Interface

### Control Panel
- **Setpoint**: Desired output voltage (0-20V)
- **Fuzzy Mode**: Enable/disable fuzzy PID adaptation
- **Manual PID**: Set Kp, Ki, Kd values manually
- **Voltage Divider**: Configure R1 and R2 values
- **Start/Stop**: Control system operation
- **Clear Data**: Reset all graphs and data

### Monitoring Panel
- **Output Voltage**: Current measured voltage
- **Error**: Current control error
- **Duty Cycle**: Current PWM duty cycle percentage
- **PID Parameters**: Current Kp, Ki, Kd values
- **Status**: System running state

### Graphs
- **Voltage Graph**: Output voltage vs setpoint over time
- **Error Graph**: Control error and PID parameters over time

## Fuzzy Rules Summary

### Kp Rules
- Large error → Higher Kp for faster response
- Small error → Lower Kp for stability
- High error rate → Lower Kp to prevent overshoot

### Ki Rules
- Persistent error → Higher Ki for steady-state accuracy
- Changing error → Lower Ki to prevent windup
- Zero error → Moderate Ki for disturbance rejection

### Kd Rules
- High error rate → Higher Kd for damping
- Low error rate → Lower Kd to reduce noise sensitivity
- Large error with high rate → Maximum Kd for stability

## Troubleshooting

### Common Issues

1. **Serial Communication Failed**
   - Check COM4 port availability
   - Verify STM32 is properly connected
   - Ensure correct baud rate (115200)

2. **Fuzzy Toolbox Error**
   - Install Fuzzy Logic Toolbox
   - Use manual PID mode as alternative

3. **Voltage Reading Issues**
   - Verify voltage divider values
   - Check ADC reference voltage (default 3.3V)
   - Ensure proper grounding

4. **Control Instability**
   - Reduce sampling rate if needed
   - Check manual PID parameters
   - Verify buck converter circuit design

### Performance Tuning

1. **Sampling Time**: Adjust based on system dynamics (default 0.1s)
2. **Fuzzy Rules**: Modify rules in `fuzzy_pid_controller.m` for specific applications
3. **PID Limits**: Adjust parameter bounds for different operating ranges
4. **Anti-windup**: Modify integral limits in `pid_controller.m`

## Technical Specifications

- **Voltage Range**: 0-20V (configurable)
- **Duty Cycle Range**: 0-100%
- **ADC Resolution**: 12-bit (0-4095)
- **Sampling Rate**: 10Hz (adjustable)
- **Communication**: 115200 baud, 8N1
- **Data Points**: 500 maximum display points

## Testing Without Hardware

Use the simulation script to test the fuzzy controller:
```matlab
test_simulation
```

This demonstrates:
- **Setpoint tracking** (5V → 8V → 3V)
- **Load disturbance rejection**
- **Fuzzy vs Manual PID comparison**
- **Performance metrics** (IAE, ISE)

## License and Support

This project is designed for educational and research purposes. For technical support or questions, refer to the MATLAB documentation for the required toolboxes.

## Version History

- **v1.0**: Initial release with fuzzy PID control, GUI interface, and serial communication 