# S U M O

This repository contains the firmware for the **STM32** microcontroller, which serves as the "Actuator" or "Muscle" of the Robot Sumo system. It is responsible for low-level hardware control, motor execution, and processing commands received from the ESP32 Gateway.

## Academic Context
This project is developed as part of a **CE232.Q21** class at the **University of Information Technology (UIT), VNU-HCM**.
## Members:
* **Le Tran Huynh Phong** (Team Lead)
* **Bui Quoc Bao**
* **Trinh Nguyen Thanh Binh**
* **Diep Thanh Phong**

## Key Features
- **Real-time Motor Control**: High-frequency PWM signals for precise movement and torque management.
- **DMA UART Communication**: High-speed command reception from the ESP32 Gateway (115200 baud).
- **Sensor Integration**: Processing data from ultrasonic sensors (opponent detection) and IR sensors (line/edge detection).

## Hardware Specifications
- **Microcontroller**: STM32F103 & ESP32s3.
- **Motor Driver**: L298N.
- **Sonar sensor**: _____.
- **IR sensor**:_________.
- ...

## Communication Protocol
The STM32 listens for instructions via UART using the following format: `COMMAND:VALUE\n`.
Example logic handled in `CMD_Process`:
- `PWR:1 / PWR:0`: System Power Enable/Disable.
- `FW:1 / BW:1`: Directional control.
- `JX:val / JY:val`: Variable speed and steering data.

