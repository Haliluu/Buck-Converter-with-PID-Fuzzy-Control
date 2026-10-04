# STM32F429 Buck Converter Implementation

## Overview
This implementation provides the STM32 firmware for the buck converter fuzzy PID control system that communicates with MATLAB via UART1.

## Hardware Configuration

### Peripherals Used
- **UART1**: Serial communication with MATLAB (115200 baud, 8N1)
- **ADC1**: Channel 0 (PA0) - Buck converter output voltage sensing
- **TIM1**: Channel 1 PWM output for buck converter switching

### Pin Configuration
- **PA0**: ADC1_IN0 - Voltage feedback (through voltage divider)
- **PA9**: UART1_TX - Transmit to PC
- **PA10**: UART1_RX - Receive from PC  
- **PE9**: TIM1_CH1 - PWM output to buck converter switch

## Communication Protocol

### Commands from MATLAB to STM32

1. **Set Duty Cycle**
   ```
   Format: DUTY:XX\n
   Example: DUTY:50\n (sets 50% duty cycle)
   Range: 0-100%
   ```

2. **Request ADC Reading**
   ```
   Format: ADC?\n
   Response: ADC:XXXX\n (where XXXX is 12-bit ADC value 0-4095)
   ```

### Response from STM32 to MATLAB

1. **ADC Value Response**
   ```
   Format: ADC:XXXX\n
   Example: ADC:2048\n (indicates ~1.65V with 3.3V reference)
   ```

2. **Startup Message**
   ```
   STM32 Buck Converter Ready\n
   ```

## Software Features

### 1. **PWM Control**
- Function: `Set_PWM_Duty(uint8_t duty)`
- Safe duty cycle limiting (0-100%)
- Real-time PWM update using `__HAL_TIM_SetCompare()`

### 2. **ADC Reading with Filtering**
- Function: `Read_ADC_Filtered(void)`
- 8-point moving average filter for noise reduction
- Polling-based ADC conversion
- Returns filtered 12-bit value (0-4095)

### 3. **UART Communication**
- Interrupt-driven receive using `HAL_UART_RxCpltCallback()`
- Command parsing and validation
- Automatic response to ADC requests
- Buffer overflow protection

### 4. **Safety Features**
- Initial duty cycle set to 0% on startup
- Duty cycle validation and limiting
- UART buffer overflow protection
- Error handling in ADC conversion

## Integration with MATLAB System

### Voltage Calculation
The MATLAB system converts ADC readings to actual voltage:
```matlab
voltage = adc_to_voltage(adc_value, R1, R2, Vref)
```

Where:
- `adc_value`: 12-bit value from STM32 (0-4095)
- `R1, R2`: Voltage divider resistors  
- `Vref`: ADC reference voltage (3.3V)

### Control Loop Timing
- MATLAB samples at 10Hz (0.1s intervals)
- STM32 responds immediately to commands
- ADC filtering provides stable readings

## Code Structure

### Main Loop
```c
while (1)
{
    Process_UART_Command();     // Handle MATLAB commands
    
    if (system_tick % 1000 == 0)
    {
        Read_ADC_Filtered();    // Periodic ADC reading
    }
    
    HAL_Delay(1);               // Prevent excessive CPU usage
}
```

### Key Functions

1. **`Initialize_System()`**
   - Sets up communication and safety defaults
   - Sends ready message to MATLAB

2. **`Process_UART_Command()`**
   - Parses incoming commands
   - Executes duty cycle changes or ADC requests

3. **`HAL_UART_RxCpltCallback()`**
   - Interrupt handler for UART reception
   - Builds command strings character by character

## Testing and Debugging

### Startup Sequence
1. STM32 initializes all peripherals
2. Sets PWM duty to 0% for safety
3. Starts UART receive interrupt
4. Sends "STM32 Buck Converter Ready" message

### Debug Features
- UART transmit function for printf debugging
- System tick counter for timing
- Status variables for monitoring

### Testing Commands
You can test the STM32 using a serial terminal:
```
DUTY:25    -> Sets 25% duty cycle
ADC?       -> Returns current ADC reading
DUTY:0     -> Sets 0% duty cycle (safe)
```

## Performance Specifications

- **PWM Frequency**: ~100kHz (configurable via TIM1 settings)
- **ADC Resolution**: 12-bit (4096 levels)
- **ADC Sampling**: On-demand with 8-point averaging
- **UART Baud Rate**: 115200 bps
- **Response Time**: <1ms for duty cycle changes
- **ADC Response Time**: <10ms including filtering

## Safety Considerations

1. **Power-on Safety**: Duty cycle starts at 0%
2. **Command Validation**: Duty cycle limited to 0-100%
3. **Communication Timeout**: MATLAB handles connection failures
4. **Emergency Stop**: Set DUTY:0 to immediately stop switching

This implementation provides a robust, real-time interface between the MATLAB fuzzy PID controller and the STM32-based buck converter hardware. 