# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F103C8 (Cortex-M3) firmware for a **self-balancing bike**. Uses STM32CubeMX-generated HAL code with FreeRTOS. The bike balances using a cascaded PID controller with MPU6050 IMU feedback, encoder speed measurement, and DC motor actuation. Includes Bluetooth telemetry (mobile app), OLED display, ultrasonic obstacle detection, and flash-based calibration storage.

## Build System

- **IDE**: Keil MDK-ARM (uVision 5), project file at [`MDK-ARM/balance_bike.uvprojx`](MDK-ARM/balance_bike.uvprojx)
- **Compiler**: ARMCC V5.06 update 6 (build 750)
- **Target**: `balance_bike`
- **Output**: [`MDK-ARM/balance_bike/balance_bike.axf`](MDK-ARM/balance_bike/balance_bike.axf) (ELF) and `.hex` (Intel HEX for flashing)
- **Defines**: `USE_HAL_DRIVER`, `STM32F103xB`
- No Makefile or CLI build — building requires Keil IDE on Windows, or adapting to ARM GCC with equivalent flags.

## Code Architecture

### Three-layer structure

1. **`Core/`** — STM32CubeMX-generated HAL init code (auto-generated, modified via `.ioc` file). Contains `main.c`, peripheral init (`adc.c`, `dma.c`, `tim.c`, `usart.c`, `gpio.c`), interrupt handlers (`stm32f1xx_it.c`), and FreeRTOS task creation (`freertos.c`). User code lives between `/* USER CODE BEGIN/END */` markers.

2. **`Drivers/`** — Vendor code: CMSIS core + STM32F1 HAL library. Not modified by application developers.

3. **`Hardwares/`** — Application-level hardware abstraction modules. Each is a self-contained `.c`/`.h` pair. This is where most development happens.

### RTOS task layout (defined in [`Core/Src/freertos.c`](Core/Src/freertos.c))

| Task | Priority | Stack | Role |
|------|----------|-------|------|
| `SensorSampleTask` | High (3) | 128 | IMU read, sensor fusion (Mahony), encoder read, PID compute, motor output |
| `ControllerTask` | AboveNormal (4) | 64 | Ultrasonic distance measurement trigger |
| `DefaultTask` | Normal (2) | 64 | LED, battery voltage ADC read, ultrasonic check |
| `BluetoothTransferTask` | Normal (2) | 128 | Bluetooth data RX/TX for mobile app |
| `DebugTransferTask` | BelowNormal (1) | 128 | ANO protocol debug serial communication |
| `ScatteredTask` | BelowNormal (1) | 64 | OLED display update (attitude, speed, distance, direction) |

Two message queues: `MpuSensorQueue` (uint32_t, depth 1) and `EncoderQueue` (uint16_t, depth 16).

The RTOS uses the **CMSIS-RTOS v1 wrapper API** (`osThreadCreate`, `osMessageCreate`, `osDelay`, etc.) — defined in `cmsis_os.h`. Static allocation is used for the idle task; heap_4.c provides dynamic allocation for everything else (4 KB heap).

### Control system (PID)

Three cascaded PID loops defined in [`Hardwares/PID/`](Hardwares/PID/):
- **`rol_angle`** — outer angle loop (keeps bike upright)
- **`vel_encoder`** — inner velocity loop (encoder feedback)
- **`rol_gyro`** — inner angular rate loop (gyro feedback)

PID compute is called from `SensorSampleTask` via `_controller_perform()` → `_controller_output()`.

### Sensor pipeline

MPU6050 (I2C via software I2C in [`Hardwares/SOFT_IIC/`](Hardwares/SOFT_IIC/)) → raw accel/gyro → low-pass filter → Mahony AHRS fusion ([`Hardwares/IMU/imu.c`](Hardwares/IMU/imu.c)) → Euler angles → PID → motor PWM (TIM3 CH1 for balance, TIM3 CH2 for forward/backward, TIM1 CH1 for steering servo).

### Hardware module inventory

| Module | Directory | Purpose |
|--------|-----------|---------|
| ANO | [`Hardwares/ANO/`](Hardwares/ANO/) | ANO protocol for debug serial |
| Bluetooth | [`Hardwares/BLUETOOTH/`](Hardwares/BLUETOOTH/) | Mobile app communication |
| Controller | [`Hardwares/CONTROLLER/`](Hardwares/CONTROLLER/) | Motor direction/steering logic |
| Display | [`Hardwares/DISPLAY/`](Hardwares/DISPLAY/) | OLED text rendering helpers |
| Encoder | [`Hardwares/ENCODER/`](Hardwares/ENCODER/) | Wheel encoder read |
| Filter | [`Hardwares/FILTER/`](Hardwares/FILTER/) | Digital filters (IIR, Butterworth) |
| Flash | [`Hardwares/FLASH/`](Hardwares/FLASH/) | STM32 internal flash for calibration data |
| IMATH | [`Hardwares/IMATH/`](Hardwares/IMATH/) | Math utilities |
| IMU | [`Hardwares/IMU/`](Hardwares/IMU/) | Mahony sensor fusion, rotation matrices |
| LED | [`Hardwares/LED/`](Hardwares/LED/) | RGB LED control (run/status indicator) |
| MPU6050 | [`Hardwares/MPU6050/`](Hardwares/MPU6050/) | IMU driver (I2C, calibration, raw→engineering units) |
| OLED | [`Hardwares/OLED/`](Hardwares/OLED/) | SSD1306 OLED low-level driver + font data |
| PID | [`Hardwares/PID/`](Hardwares/PID/) | PID controller structure + 3-loop init |
| Servo | [`Hardwares/SERVO/`](Hardwares/SERVO/) | Steering servo |
| Soft IIC | [`Hardwares/SOFT_IIC/`](Hardwares/SOFT_IIC/) | Bit-banged I2C for MPU6050 |
| Ultrasonic | [`Hardwares/ULTRASONIC/`](Hardwares/ULTRASONIC/) | HC-SR04 distance sensor |

### Pin configuration

Pin defines are in [`Core/Inc/main.h`](Core/Inc/main.h). Key assignments:
- **LED**: PC13
- **Debug UART (USART1)**: PA9 (TX), PA10 (RX)
- **Bluetooth UART (USART2)**: PA2 (TX), PA3 (RX)
- **Motor**: PB8 (EN), PB9 (DIR), PWM on TIM3 CH1/CH2
- **Servo**: PWM on TIM1 CH1
- **MPU6050 I2C**: PB6 (SCL), PB7 (SDA) — software I2C
- **OLED I2C**: PB12 (SCL), PB13 (SDA) — software I2C (shared Soft IIC module)
- **Ultrasonic**: PB10 (TRIG), PB11 (ECHO, EXTI)
- **Battery ADC**: PA4 (ADC1 CH4)

### FreeRTOS configuration

[`Core/Inc/FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h): Preemptive scheduler, 1000 Hz tick, 7 priority levels, 64-word min stack, 4 KB total heap (heap_4), static + dynamic allocation, ARM Cortex-M3 port.

### STM32CubeMX regeneration

The project was generated by STM32CubeMX ([`balance_bike.ioc`](balance_bike.ioc)). When regenerating:
- Only `Core/` files are overwritten (between USER CODE markers)
- `Hardwares/` is application code and never touched by CubeMX
- Include paths for `Hardwares/` modules must be manually added to the Keil project

## Pin naming conventions

Chinese-language comments use the following conventions:
- 平衡 = balance
- 方向 = direction
- 速度 = speed/velocity
- 角度 = angle
- 电机 = motor
- 舵机 = servo/steering
- 超声波 = ultrasonic
- 编码器 = encoder
- 陀螺仪 = gyroscope
- 加速度 = acceleration
