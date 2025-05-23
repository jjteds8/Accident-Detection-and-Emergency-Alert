# Car Accident Detection & Alert System  
**for LilyGO T-A7670E ESP32**

---

## Overview

This project is an all-in-one **Car Accident Detection & Alert System** built for the LilyGO T-A7670E ESP32 board, supporting:

- **Accident detection** using G-force (MPU6050) and orientation
- **SMS alerts** via A7670E cellular module (Smart, Globe, etc.)
- **Web dashboard** for live sensor data, GPS maps, and settings
- **Proximity warning** (buzzer beeps when object is close)
- **Configurable thresholds** (impact, flip, proximity) via web
- **Easy WiFi AP access** (dashboard at `192.168.4.1`)
- **Manual alert trigger** (via web, not hardware button)

---

## Features

- **Accident Sensing**
  - Detects high G-force or abnormal orientation (flipped/on side)
  - Sends emergency SMS with location map link

- **Live Sensor Dashboard**
  - G-force, orientation, GPS, proximity, buzzer & alert status

- **GPS Integration**
  - NEO-6M GPS for latitude, longitude, speed
  - Google Maps embed and SMS link

- **Cellular SMS**
  - Uses A7670E (SIMCOM) for SMS
  - Works with Smart/Globe SIM (requires SMS balance)

- **Web UI**
  - View sensor data, map, settings, and test triggers
  - Change thresholds, recipient number, and test SMS/buzzer

- **Buzzer Proximity Alert**
  - Buzzer beeps when object is closer than set threshold

- **Manual SMS Trigger**
  - Trigger alert via web dashboard (hardware button disabled to prevent false triggers)

---

## Hardware Requirements

- **LilyGO T-A7670E ESP32** (A7670E cellular + ESP32)
- **MPU6050** (accelerometer/gyro, I2C)
- **NEO-6M GPS** module (UART)
- **Ultrasonic sensor** (HC-SR04 or compatible)
- **Buzzer** (active type, digital pin)
- **Button** (for reset only, not manual alert)
- **SIM Card** (with SMS credits, e.g. Smart/Globe PH)

---

## Pin Connections

| Module         | ESP32 Pin      | Notes                  |
|----------------|--------------- |------------------------|
| A7670E RX      | GPIO 26        | UART1 TX               |
| A7670E TX      | GPIO 27        | UART1 RX               |
| PWRKEY         | GPIO 4         | Power control          |
| POWERON        | GPIO 12        |                        |
| RESET          | GPIO 5         |                        |
| Buzzer         | GPIO 14        | Output                 |
| Button         | GPIO 13        | Input, pullup          |
| Ultrasonic TRIG| GPIO 19        | Output                 |
| Ultrasonic ECHO| GPIO 18        | Input                  |
| GPS RX         | GPIO 33        | UART2 TX (to GPS)      |
| GPS TX         | GPIO 22        | UART2 RX (from GPS)    |
| I2C (MPU6050)  | SDA/SCL        | Default ESP32 pins     |

---

## Usage

### 1. **Upload the Code**

- Use PlatformIO or Arduino IDE.
- Install required libraries:
  - `Adafruit MPU6050`
  - `Adafruit Unified Sensor`
  - `TinyGPSPlus`
  - `WiFi`
  - `WebServer`

### 2. **Power On**

- Insert a SIM with SMS credit.
- Power the device (battery or USB).
- The device starts a WiFi AP:  
  SSID: `CAR_ALERT_HOTSPOT`  
  Password: `caralert123`

### 3. **Connect to Dashboard**

- Connect your phone/laptop to the WiFi AP.
- Open [http://192.168.4.1](http://192.168.4.1) in your browser.

### 4. **Configure & Test**

- **Set recipient number** for SMS alerts (international format, e.g. `+639XXXXXXXXX`).
- **Adjust thresholds** as needed.
- **Test SMS** and **buzzer** via the web UI.
- View live sensor data and GPS location.

### 5. **Operation**

- The system automatically sends SMS alerts on high G-force, flipping, or manual trigger via web UI.
- Buzzer sounds when an object is close (ultrasonic).
- Use the button for **reset only** (long press, 10 seconds).

---

## Web Dashboard Overview

- **Home:** Quick links and system overview
- **Sensor Data:** Live readings of all sensors and system state
- **GPS Map:** View current GPS location on map
- **Manual/Test:** Trigger test SMS/buzzer, manual alert (web only), and reset
- **Settings:** Change alert thresholds, recipient number, view SIM/cellular status

---

## Example SMS Alert

```
CAR ACCIDENT ALERT!
Cause: High G
G's: 7.40
Location: https://maps.google.com/?q=14.6091,121.0223
State: Flipped
```

---

## Notes

- **Manual alert via hardware button is disabled** to prevent false triggers.
- Use web dashboard for all manual testing/triggers.
- Ensure your SIM can send SMS and has sufficient load/credit.
- If SMS sending fails, check the debug log on the web dashboard for AT command errors.

---

## License

MIT (c) 2025 jjteds8

---