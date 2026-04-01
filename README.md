# ACR-MK1 (Autonomous Courier Robot) 🤖📍

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry%20Pi-C51A4A.svg)
![Flask](https://img.shields.io/badge/Framework-Flask-black.svg)

**ACR-MK1** (Autonomous Courier Robot - Mark 1) is a fully-featured, Raspberry Pi-based teleoperated and semi-autonomous robot. It features a rich web-based dashboard for real-time telemetry (GPS tracking, compass heading, camera feed) alongside a terminal-based WASD control interface. It includes built-in obstacle avoidance and dynamic physics for smooth acceleration and steering.

---

## 🚀 Key Features

* **Real-Time Web Dashboard:** Built with Flask, serving a live dashboard at port `5000`.
* **Live Camera Stream:** Low-latency video streaming using `Picamera2` and OpenCV.
* **Live GPS Mapping:** Integration with Leaflet.js to plot the robot's coordinates on a map in real-time. Parses NMEA sentences via UART.
* **Advanced Telemetry:** Auto-detects and reads from multiple I2C magnetometers (HMC5883L, QMC5883L, IST8310) to calculate precise headings. Features a web-triggered calibration sequence.
* **Smart Obstacle Avoidance:** Integrates with 5 MZ80 infrared sensors (Front, Left-Front, Right-Front, Left-Rear, Right-Rear). Automatically cuts forward throttle if an obstacle is detected.
* **Smooth WASD Teleoperation:** Terminal-based control over SSH utilizing a custom physics model (acceleration, coasting, and braking rates) for natural movement.

---

## 🛠 Hardware Requirements

* **SBC:** Raspberry Pi (3, 4, or 5 recommended) running Raspberry Pi OS (Bullseye or newer for Picamera2 support).
* **Motors:** 2x DC Motors with a Motor Driver (e.g., L298N).
* **Sensors:**
  * 5x MZ80 Infrared Obstacle Avoidance Sensors.
  * 1x GPS Module (UART interface).
  * 1x I2C Magnetometer/Compass (HMC5883L, QMC5883L, or IST8310).
* **Camera:** Raspberry Pi Camera Module.

---

## 🔌 Pin Configuration (BCM)

### Motor Driver
| Component | RPi GPIO (BCM) | Description |
| :--- | :--- | :--- |
| **PWM1** | `GPIO 18` | Left Motor Speed (PWM) |
| **DIR1** | `GPIO 23` | Left Motor Direction |
| **PWM2** | `GPIO 13` | Right Motor Speed (PWM) |
| **DIR2** | `GPIO 6` | Right Motor Direction |

### MZ80 Obstacle Sensors
*Sensors use internal Pull-Up resistors (`PUD_UP`). Logic LOW (`0`) triggers the obstacle flag.*

| Sensor Position | RPi GPIO (BCM) |
| :--- | :--- |
| **Front** | `GPIO 17` |
| **Left Front** | `GPIO 16` |
| **Right Front** | `GPIO 22` |
| **Left Rear** | `GPIO 5` |
| **Right Rear** | `GPIO 26` |

### I2C & UART Connections
* **GPS:** Connect to RPi Serial TX/RX (`/dev/serial0`, `/dev/ttyAMA0`, or `/dev/ttyS0`).
* **Compass:** Connect to RPi I2C pins (SDA1, SCL1).

---

## 📦 Installation & Setup

**1. Clone the repository**
```bash
git clone https://github.com/burakdevelopment/acr-mk1.git
cd acr-mk1
```

**2. Enable Hardware Interfaces**
Ensure Camera, I2C, and Serial are enabled on your Raspberry Pi:
```bash
sudo raspi-config
```
* Navigate to `Interface Options`.
* Enable **I2C**.
* Enable **Serial Port** (Disable serial login shell, but enable hardware serial).
* Enable **Camera** (Legacy or libcamera depending on your OS).

**3. Install System Dependencies**
```bash
sudo apt-get update
sudo apt-get install python3-opencv python3-smbus i2c-tools
```

**4. Install Python Dependencies**
```bash
pip3 install Flask RPi.GPIO pyserial pynmea2 smbus2
```
*(Note: `picamera2` is usually pre-installed on modern Raspberry Pi OS releases. If not, refer to official PiCamera2 documentation).*

---

## 🎮 Usage

Run the script from your terminal (preferably over SSH). **You must run this with `sudo` privileges to access GPIO pins.**

```bash
sudo python3 courierv2.py
```

### 1. Web Dashboard
Once the script is running, open a web browser on any device connected to the same network and navigate to:
```text
http://<RASPBERRY_PI_IP>:5000
```
Here you can view the live camera feed, track the ACR-MK1 on the GPS map, monitor telemetry, and calibrate the compass.

### 2. WASD Terminal Controls
The robot is controlled directly from the terminal session where you executed the script. The controls use non-blocking inputs.

| Key | Action |
| :---: | :--- |
| **W** | Accelerate / Throttle |
| **S** | Brake / Decelerate |
| **A** | Steer Left |
| **D** | Steer Right |
| **R** | Toggle Gear (Forward / Reverse) |
| **SPACE** | Emergency Stop (Sets velocity to 0) |
| **Q** / **Ctrl+C** | Quit application safely |

---

## 📡 REST API Endpoints

The Flask server exposes several JSON endpoints for fetching telemetry externally:

* `GET /api/gps` - Returns current Latitude and Longitude.
* `GET /api/gps_detail` - Returns verbose GPS data (Sats, HDOP, Alt, Speed).
* `GET /api/compass` - Returns magnetometer data and heading.
* `GET /api/telemetry` - Combined payload of GPS and Compass data.
* `POST /api/compass/calibration/start` - Initiates compass calibration.
* `POST /api/compass/calibration/stop` - Ends calibration and saves offsets to `/home/pi/mag_calibration.json`.

---

## ⚠️ Safety Notes
* Ensure the robot is elevated (wheels off the ground) during initial testing to prevent unexpected runaways.
* Obstacle detection automatically cuts the forward throttle. Reversing is still permitted to back away from the obstacle.
* The script safely cleans up GPIO pins and stops PWM signals upon exit (`Ctrl+C`).
```
