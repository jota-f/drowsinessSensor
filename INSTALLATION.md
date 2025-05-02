# drowsinessSensor Installation Guide

This guide details how to set up and install the drowsinessSensor drowsiness detection system for drivers.

## Hardware Requirements

- ESP32-CAM (AI-Thinker model recommended)
- Buzzer (active, without oscillator)
- FTDI adapter or USB-Serial converter for programming
- 5V/2A power supply
- Jumper wires for connections
- Optional: Protective case for ESP32-CAM

## Arduino Environment Setup

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 support:
   - Open Arduino IDE
   - Go to File > Preferences
   - Under "Additional Board Manager URLs", add:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Click OK
   - Go to Tools > Board > Boards Manager
   - Search for "ESP32"
   - Install "ESP32 by Espressif Systems"

## Hardware Configuration

### Programming Connections

1. Connect the ESP32-CAM to the FTDI/USB-Serial converter:
   - ESP32-CAM GND → FTDI GND
   - ESP32-CAM 5V → FTDI VCC (5V)
   - ESP32-CAM U0R (TX) → FTDI RX
   - ESP32-CAM U0T (RX) → FTDI TX
   - ESP32-CAM IO0 → FTDI GND (only during upload)

2. Connect the buzzer:
   - Buzzer (+) → ESP32-CAM IO12
   - Buzzer (-) → ESP32-CAM GND

### Connection Diagram

```
┌─────────────┐                  ┌─────────────┐
│             │                  │             │
│   ESP32-CAM │                  │    FTDI     │
│             │                  │             │
│         GND ├──────────────────┤ GND         │
│          5V ├──────────────────┤ VCC (5V)    │
│         U0R ├──────────────────┤ RX          │
│         U0T ├──────────────────┤ TX          │
│         IO0 ├─┬────────────────┤             │
│         IO12├─┼────────────┐   │             │
│             │ │            │   │             │
└─────────────┘ │            │   └─────────────┘
                │            │
                │            │   ┌─────────────┐
                │            │   │             │
                │            └───┤ +   BUZZER  │
                └────────────────┤ -           │
                                 │             │
                                 └─────────────┘
```

**Note:** The connection between IO0 and GND is only necessary during code upload. Remove after uploading.

## Software Installation

1. Download the repository to your computer
2. Open the `drowsinessSensor.ino` file in Arduino IDE
3. Import the Edge Impulse library:
   - Go to Sketch > Include Library > Add .ZIP Library
   - Select the `ei-snorless-1-arduino-1.0.7.zip` file included in this repository

4. Configure Arduino IDE:
   - Tools > Board > ESP32 Arduino > AI Thinker ESP32-CAM
   - Tools > Port > Select the COM port of your FTDI adapter
   - Tools > Partition Scheme > Huge APP (3MB No OTA/1MB SPIFFS)
   - Tools > Upload Speed > 115200

5. Prepare the ESP32-CAM for upload:
   - Connect the IO0 pin to GND (programming mode)
   - Press the RESET button on the ESP32-CAM board
   - Click "Upload" in Arduino IDE
   - After upload, disconnect IO0 from GND
   - Press RESET again

## Positioning and Usage

1. Mount the ESP32-CAM in a position that captures the driver's face:
   - Ideally on the dashboard, centered
   - The camera should be facing the face, with a clear view
   - Avoid direct strong light on the camera
   - Recommended distance: 30-60 cm from the face

2. Connect to power (5V):
   - We recommend using a stable 5V/2A power supply
   - Or connect to a vehicle's USB port with an appropriate adapter

3. When turning on the system:
   - The red LED will light up during initialization
   - After a few seconds, the system will start monitoring
   - No additional configuration is necessary

## Troubleshooting

- **LED blinks repeatedly and system doesn't initialize:**
  - Check the power supply (stable 5V)
  - Try using a different power source

- **Camera doesn't initialize:**
  - Press the RESET button
  - Check the power supply
  - Reconnect the system

- **Inconsistent detections:**
  - Improve face lighting
  - Adjust camera position
  - Make sure the face is completely visible

- **System freezes:**
  - Press the RESET button
  - If the problem persists, check the power supply

## Maintenance

- Clean the camera lens periodically with a soft cloth
- Check connections regularly
- Keep the system free from dust and moisture 