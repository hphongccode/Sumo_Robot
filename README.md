# Robot Sumo - ESP32 Gateway (Master Controller)

This branch contains the source code for the **ESP32-S3**, acting as the Gateway (Master Controller) in a multi-MCU Robot Sumo system. The ESP32 handles Cloud connectivity, remote control, and resource monitoring, then sends execution commands to the STM32.

## Key Features
- **Blynk IoT Integration**: Pure MQTT protocol communication for remote control via smartphone.
- **Dual-MCU Communication**: Sends control commands to the STM32 via UART (115200 baud).
- **Local OTA (mDNS)**: Over-The-Air firmware updates over the local network without needing a USB cable.

## Hardware Pinout

| Peripheral | ESP32-S3 Pin | Component Pin | Notes |
| :--- | :--- | :--- | :--- |
| **UART (STM32)** | GPIO 17 (TX) | RX (STM32) | Sends control commands |
| **UART (STM32)** | GPIO 18 (RX) | TX (STM32) | Receives feedback |

## UART Command Protocol
Data sent from the ESP32 to the STM32 follows a strict string format: `COMMAND:VALUE\n` </br>
For examples: 
- `FW:1` / `FW:0`: Move Forward / Stop.
- `BW:1` / `BW:0`: Move Backward / Stop.
- `JX:val` / `JY:val`: Analog Joystick values mapped from Blynk.
- `PWR:1` / `PWR:0`: Power ON / OFF the motor drive system.

