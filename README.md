# Forklift Control System using ESP32 and Dabble

## Overview
This project implements a forklift control system using an ESP32 microcontroller, a stepper motor, and DC motors. It is controlled via the Dabble app, specifically using the GamePad module over Bluetooth. The system supports movement control, stepper motor operation, recording and playback of motor actions, and an emergency stop feature.

## Features
- **Bluetooth Control**: Uses the Dabble app for wireless control.
- **Motor Control**: Controls two DC motors for movement and a stepper motor for lifting operations.
- **Recording & Playback**: Allows recording and playback of motor movements.
- **Emergency Stop**: A cross-button press halts all operations immediately.
- **Automatic Return**: Returns the stepper motor to its initial position before playback.

## Hardware Requirements
- **ESP32 Development Board**
- **Stepper Motor (28BYJ-48) with ULN2003 Driver**
- **Four DC Motors**
- **Motor Driver Module (L298N or similar)**
- **Dabble App (Installed on Smartphone)**
- **Power Supply** (as required for the motors and ESP32)

## Wiring Configuration
### Stepper Motor (28BYJ-48)
| ESP32 Pin | Stepper Motor Driver |
|-----------|----------------------|
| 12        | IN1                  |
| 14        | IN2                  |
| 27        | IN3                  |
| 26        | IN4                  |

### DC Motor Control (L298N)
| ESP32 Pin | Function |
|-----------|----------|
| 22        | Enable Right Motor |
| 16        | Right Motor IN1 |
| 17        | Right Motor IN2 |
| 23        | Enable Left Motor |
| 18        | Left Motor IN1 |
| 19        | Left Motor IN2 |

## Software Setup
### Required Libraries
Install the following Arduino libraries:
- `Stepper.h` (for stepper motor control)
- `DabbleESP32.h` (for Bluetooth control via Dabble)

### Uploading the Code
1. Install the required libraries in the Arduino IDE.
2. Connect the ESP32 board to your computer via USB.
3. Select the appropriate board and port in the Arduino IDE.
4. Compile and upload the code.

## Controls (Using Dabble GamePad)
| Button  | Function |
|---------|----------|
| Up Arrow | Move Forward |
| Down Arrow | Move Backward |
| Left Arrow | Turn Left |
| Right Arrow | Turn Right |
| Triangle | Move Stepper Motor Up |
| Square | Move Stepper Motor Down |
| Select | Start/Stop Recording Movements |
| Start | Start/Stop Playback |
| Cross | Emergency Stop |

## Functionality
### 1. **Motor Control**
- The user can move the forklift using the arrow buttons.
- The stepper motor can be controlled using the Triangle (up) and Square (down) buttons.

### 2. **Recording Movements**
- Press the **Select** button to start recording motor movements.
- Press the **Select** button again to stop recording.

### 3. **Playback Movements**
- Press the **Start** button to play back recorded movements.
- The stepper motor returns to its initial position before playback begins.

### 4. **Emergency Stop**
- Press the **Cross** button to immediately stop all operations.

## Debugging & Monitoring
- Open the **Serial Monitor** in the Arduino IDE at **115200 baud** to view debug messages.

## Future Enhancements
- Implement more refined movement smoothing.
- Add a display to show real-time movement status.
- Extend the project to use additional sensors for automation.

## License
This project is open-source and can be freely used and modified.

