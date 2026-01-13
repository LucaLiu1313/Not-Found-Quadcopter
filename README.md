# Not-Found Quadcopter

This is a bare-metal STM32F401 flight controller project focused on attitude estimation and PID control.

### 👥 Core Contributors

[![Fengran Zhao](https://img.shields.io/badge/Core--Contributor-Fengran_Zhao-blue?style=flat-square)](https://github.com/kcordum)
[![Jinliang Liu](https://img.shields.io/badge/-Jinliang_Liu-blue?style=flat-square)](https://github.com/LucaLiu1313)
[![Zijie Liu](https://img.shields.io/badge/-Zijie_Liu-blue?style=flat-square)](https://github.com/2024lzj)
[![Tongyu Ye](https://img.shields.io/badge/-Tongyu_Ye-blue?style=flat-square)](https://github.com/wutongX1453)
## Overview
- Sensor fusion: MPU6050 (accel + gyro) + HMC5883L (magnetometer)
- Attitude solution: quaternion + Madgwick gradient correction
- Cascaded PID: attitude outer loop + rate inner loop
- RC input: PPM (8 channels)
- Motor output: 4-channel PWM (ESC)
- Debug: OLED and UART

## Directory Layout
- `User/`: main flow and PID logic (`main.c`, `pid/`)
- `imu/`: attitude estimation core (quaternion update, Euler output)
- `Hardware/`: drivers (PPM, PWM, I2C, IMU, OLED, UART)
- `CMSIS/`: startup and system init
- `Library/`: STM32F4 StdPeriph library
- `Project/`: Keil MDK project and build artifacts

## Hardware and Pin Map
**MCU and sensors**
- MCU: STM32F401RETx
- IMU: MPU6050 + HMC5883L (e.g. GY-86)
- OLED: I2C display (SSD1306 common, address 0x78)
- RC: PPM receiver (8 channels)

**Pins (from driver sources)**
- PPM input: `PA1` / `TIM2_CH2` (`Hardware/PPM.c`)
- PWM output (4 motors): `TIM3_CH1..4`
  - `PA6` (CH1), `PA7` (CH2), `PB0` (CH3), `PB1` (CH4)
- IMU I2C (bit-bang): `PB8` (SCL), `PB9` (SDA)
- OLED I2C (bit-bang): `PB10` (SCL), `PB3` (SDA)
- UART debug: `USART1` -> `PA9` (TX), `PA10` (RX)

> Note: The I2C buses are bit-banged open-drain. Make sure pull-ups and power are stable.

## Boot and Control Flow (Bare Metal)
1. Init peripherals: OLED, UART, I2C, PWM, MPU6050, HMC5883L, PPM, PID
2. ESC calibration: output 2000us then 1000us (hard-coded in `User/main.c`)
3. IMU bias calibration: 50 samples at rest (`imu/imu.c: err_update`)
4. Main loop:
   - Attitude: `IMU_update` -> `pitch/roll/yaw`
   - Target: `getWantedYPR` from PPM (angle or yaw-rate)
   - Cascaded PID: attitude + rate loops
   - Mixer: throttle + PID corrections -> 4 PWM outputs
   - OLED/UART output (optional)

## Build and Flash (Bare Metal)
**Keil MDK**
1. Open `Project/Project.uvprojx`
2. Select `Target 1` (device `STM32F401RETx`)
3. Build & Download

The project uses the STM32F4 StdPeriph library (`Library/`) and CMSIS startup files.

## Tuning Entry Points
**RC mapping and deadzone**
- File: `User/pid/control.c`
- Macros: `RC_CH_*`, `RC_PULSE_MID`, `RC_PULSE_SPAN`, `RC_DEADZONE`
- Limits: `ROLL_MAX_DEG`, `PITCH_MAX_DEG`, `YAW_RATE_MAX_DEGPS`

**PID gains**
- File: `User/pid/pid.c`
- Outer loop: `pramOut`
- Inner loop: `pramIn`
- Integrator caps and split thresholds: `ITHRE`, `posISepThre`, `rateIsepThre`

**IMU and fusion**
- File: `imu/imu.c`
- Sample period: `halfT` (dt = 2 * halfT)
- Madgwick parameter: `beta`

**PPM input and PWM timing**
- PPM capture: `Hardware/PPM.c` (valid range and frame sync)
- PWM timing: `Hardware/pwm.c` (TIM3 PSC/ARR)

## UART Output
UART defaults to 9600 baud (`Hardware/Serial.c`). `User/main.c` includes a JSON-style telemetry snippet you can enable for plotting or tuning.

## Safety Notes
- Remove propellers during bench testing.
- Always validate safety before flight.
