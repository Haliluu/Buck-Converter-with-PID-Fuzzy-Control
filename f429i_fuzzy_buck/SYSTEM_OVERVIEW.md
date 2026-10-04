# Buck Converter Adaptive PID Fuzzy Control System
## Complete System Overview

This project implements a complete buck converter control system using adaptive fuzzy PID control with real-time communication between MATLAB and STM32F429.

---

## 🎯 System Architecture

```
┌─────────────────┐    UART COM4     ┌──────────────────┐    PWM    ┌─────────────────┐
│                 │   115200 baud    │                  │  Signal   │                 │
│   MATLAB GUI    │ ←──────────────→ │    STM32F429     │ ────────→ │ Buck Converter  │
│  Fuzzy PID      │                  │   PWM + ADC      │           │    Circuit      │
│                 │                  │                  │ ←──────── │                 │
└─────────────────┘                  └──────────────────┘  Voltage  └─────────────────┘
                                                          Feedback
```

---

## 📁 Project Structure

```
f429i_fuzzy_buck/
├── matlab/                          # MATLAB Implementation
│   ├── main.m                       # System startup script
│   ├── buck_converter_gui.m         # Main GUI with integrated serial comm
│   ├── fuzzy_pid_controller.m       # Fuzzy logic for PID adaptation
│   ├── pid_controller.m             # PID controller with anti-windup
│   ├── test_simulation.m            # Hardware-less simulation
│   ├── test_functions.m             # System verification script
│   └── README.md                    # MATLAB documentation
├── Core/Src/main.c                  # STM32 firmware
├── STM32_Implementation_Notes.md    # STM32 documentation
└── SYSTEM_OVERVIEW.md              # This file
```

---

## 🚀 Quick Start Guide

### 1. **Hardware Setup**
- Connect STM32F429 to PC via USB (appears as COM4)
- Wire buck converter circuit:
  - **PE9** → PWM to buck converter switch
  - **PA0** → Voltage feedback via voltage divider (R1=1kΩ, R2=1kΩ)
  - **PA9/PA10** → UART communication

### 2. **STM32 Programming**
- Open STM32CubeIDE
- Build and flash `Core/Src/main.c`
- Verify "STM32 Buck Converter Ready" message on startup

### 3. **MATLAB Operation**
```matlab
cd matlab
main          % Launch the system
```
- Configure voltage setpoint and parameters in GUI
- Click **Start** to begin control
- Monitor real-time graphs and PID adaptation

---

## 🧠 Fuzzy Logic Implementation

### **Input Variables**
- **Error**: `setpoint - actual_voltage` (normalized ±1)
- **Error Rate**: `d(error)/dt` (normalized ±1)

### **Output Variables**
- **Kp**: Proportional gain (0.1 - 10)
- **Ki**: Integral gain (0.01 - 5)
- **Kd**: Derivative gain (0.001 - 2)

### **Fuzzy Sets**
- **5 membership functions** per input/output
- **75 total rules** (25 per parameter)
- **Mamdani inference** with centroid defuzzification

### **Adaptation Strategy**
- **Large error** → Higher Kp for fast response
- **Persistent error** → Higher Ki for accuracy
- **High error rate** → Higher Kd for damping

---

## 🔧 System Specifications

### **MATLAB Side**
| Parameter | Value | Description |
|-----------|-------|-------------|
| **Sampling Rate** | 10 Hz | Control loop frequency |
| **Communication** | COM4, 115200 baud | Serial interface |
| **Voltage Range** | 0-20V | Configurable setpoint range |
| **Data Points** | 500 max | Real-time display buffer |

### **STM32 Side**
| Parameter | Value | Description |
|-----------|-------|-------------|
| **PWM Frequency** | ~100 kHz | Buck converter switching |
| **ADC Resolution** | 12-bit | Voltage measurement |
| **ADC Filtering** | 8-point moving average | Noise reduction |
| **Response Time** | <1 ms | Command execution |

---

## 📡 Communication Protocol

### **MATLAB → STM32**
```
DUTY:XX\n     % Set PWM duty cycle (0-100%)
ADC?\n        % Request voltage reading
```

### **STM32 → MATLAB**
```
ADC:XXXX\n    % Send 12-bit ADC value (0-4095)
```

### **Voltage Conversion**
```matlab
actual_voltage = (adc_value/4095) * 3.3V * (R1+R2)/R2
```

---

## 🛡️ Safety Features

### **STM32 Safety**
- ✅ **Power-on protection**: 0% duty cycle on startup
- ✅ **Input validation**: Duty cycle limited to 0-100%
- ✅ **Communication timeout**: Graceful error handling
- ✅ **Buffer protection**: UART overflow prevention

### **MATLAB Safety**
- ✅ **Parameter bounds**: PID values within safe ranges
- ✅ **Anti-windup**: Integral term limiting
- ✅ **Emergency stop**: Immediate 0% duty on stop
- ✅ **Connection monitoring**: Serial port status tracking

---

## 📊 Performance Features

### **Real-time Monitoring**
- 📈 **Voltage tracking**: Output vs setpoint graphs
- 📉 **Error analysis**: Control error visualization
- 🎛️ **PID parameters**: Live adaptation display
- 📋 **System status**: Connection and control state

### **Advanced Control**
- 🧠 **Fuzzy adaptation**: Automatic PID tuning
- 🎯 **Setpoint tracking**: Dynamic reference following
- 🔄 **Disturbance rejection**: Load change compensation
- 📊 **Performance metrics**: IAE, ISE calculation

---

## 🧪 Testing & Validation

### **Without Hardware**
```matlab
test_simulation    % Run buck converter simulation
test_functions     % Verify all components
```

### **With Hardware**
1. **Connection test**: Check COM4 communication
2. **Safety test**: Verify 0% startup duty
3. **Response test**: Manual duty cycle commands
4. **Control test**: Closed-loop fuzzy PID operation

---

## 🔬 Technical Highlights

### **Fuzzy Logic Benefits**
- **Adaptive behavior**: PID parameters adjust to operating conditions
- **Robust performance**: Handles nonlinearities and uncertainties
- **Fast settling**: Optimal response for different error magnitudes
- **Stable operation**: Reduced overshoot and oscillations

### **System Integration**
- **Modular design**: Separate GUI, control, and communication functions
- **Real-time operation**: Sub-millisecond response times
- **Scalable architecture**: Easy parameter and rule modifications
- **Professional interface**: Publication-ready visualizations

---

## 🛠️ Customization Options

### **Fuzzy Rules Tuning**
Modify `fuzzy_pid_controller.m` to adjust:
- Membership function shapes
- Rule base consequences
- Input/output scaling factors

### **Control Parameters**
Adjust in GUI or code:
- Sampling time (0.05s - 1.0s)
- PID parameter bounds
- Voltage divider values
- Setpoint limits

### **Hardware Configuration**
Change in STM32 code:
- PWM frequency (TIM1 settings)
- ADC channels and pins
- Communication baud rate
- Filter parameters

---

## 📈 Expected Results

### **Performance Metrics**
- **Settling time**: <2 seconds for step changes
- **Steady-state error**: <1% of setpoint
- **Overshoot**: <5% with fuzzy adaptation
- **Disturbance rejection**: <10% voltage droop

### **Fuzzy vs Manual PID**
- **IAE improvement**: 15-30% better
- **ISE improvement**: 20-40% better
- **Robustness**: Superior across operating ranges
- **Tuning effort**: Minimal manual adjustment needed

---

## 🎓 Educational Value

This project demonstrates:
- **Advanced Control Theory**: Fuzzy logic, PID, adaptive systems
- **Real-time Systems**: Hardware-software integration
- **Professional Tools**: MATLAB, STM32, GUI development
- **System Engineering**: Complete design cycle from theory to implementation

Perfect for:
- Control systems courses
- Power electronics projects
- Graduate research
- Industrial applications

---

## 📞 Support & Documentation

- **MATLAB Help**: See `matlab/README.md`
- **STM32 Details**: See `STM32_Implementation_Notes.md`
- **Function Testing**: Run `test_functions.m`
- **Simulation Demo**: Run `test_simulation.m`

---

**Ready to revolutionize your buck converter control! 🚀** 