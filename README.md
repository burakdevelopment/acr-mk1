# ACR-MK1 — Autonomous Courier Robot (Raspberry Pi 4 + Pi Camera v3 + M8N GPS/Compass)

**ACR-MK1** (*Autonomous Courier Robot*) is a compact courier robot platform built around **Raspberry Pi 4 (8GB)** and **Raspberry Pi Camera Module v3**, designed to operate anywhere with cellular coverage (via USB 4G modem) and provide a solid, production-style base for full autonomy.

This repository focuses on a **robust robotics backbone**:
- **Live camera streaming (MJPEG)**
- **Live GPS tracking on a web map (Leaflet + OSM)**
- **Magnetometer compass heading + web-based calibration flow**
- **Manual driving via gamepad**
- **Obstacle safety stop via MZ80 IR sensors (5x)**
- Clean structure that allows adding autonomy modules (perception → planning → control) without breaking the base stack.

---

## Hardware Overview (Current Build)

### Robot Preview Images

<img width="3024" height="4032" alt="Untitled2(1)" src="https://github.com/user-attachments/assets/f8b73add-6a07-475c-963b-03e02f71a18d" />

<img width="2160" height="3840" alt="2" src="https://github.com/user-attachments/assets/99524712-75c9-4d5f-af1f-0a78309d3394" />


### Core Compute
- **Raspberry Pi 4 Model B — 8GB RAM**
- **Raspberry Pi OS** (Bookworm/Bullseye compatible)
- **Raspberry Pi Camera Module v3**

### Navigation Sensor Module
- **u-blox NEO-M8N GNSS module** (GPS/GLONASS/etc.)
- **Magnetometer (Compass)** on the same module (commonly **QMC5883L** or **HMC5883L**)

### Obstacle Safety
- **MZ80 IR obstacle sensors (5 units)**
  - 1x front-center
  - 2x left side
  - 2x right side

### Drive System
- 4x DC motors (4-wheel drive)
- Motor driver(s) compatible with PWM + direction pins (H-bridge style)
- **Two PWM channels** used because left motors are wired in parallel and right motors are wired in parallel (differential steering)

### Battery
- **10A LiPo** (ensure the pack’s **C-rating** and capacity match your current draw — see Power section)

### Optional Connectivity
- USB **4G/LTE modem** (SIM dongle) for remote monitoring/control anywhere with base station coverage

---

## Wiring / Pinout

> GPIO numbering below uses **BCM** (not physical pin numbers).

### Motor Driver (Differential — Left/Right)
| Function | Raspberry Pi GPIO (BCM) | Notes |
|---|---:|---|
| PWM Left | `GPIO18` | PWM signal to left motor driver input |
| DIR Left | `GPIO23` | Direction pin (HIGH/LOW depends on driver wiring) |
| PWM Right | `GPIO13` | PWM signal to right motor driver input |
| DIR Right | `GPIO6`  | Direction pin |

**Why 2 PWM?**  
Left motors are parallel on one driver channel, right motors parallel on another. Steering is achieved by reducing PWM on one side (differential).

---

### MZ80 Obstacle Sensors (5x)
The code currently uses **one** safety input:
| Sensor | Raspberry Pi GPIO (BCM) | Notes |
|---|---:|---|
| Front-center | `GPIO17` | Configured with internal pull-up. Sensor outputs LOW when obstacle detected (typical). |

✅ Your build has 5 sensors (2 left, 2 right, 1 front).  
This repo’s next step is to map all 5 into:
- Front: hard stop
- Sides: soft correction / wall-follow / slow-down

(Implementation is designed to be added without breaking the current backbone.)

---

### M8N GNSS + Magnetometer Module (as you wired)
You connected:
- **VCC → +5V**
- **GND → GND**
- **SDA → GPIO2 (I2C SDA)**
- **SCL → GPIO3 (I2C SCL)**
- **TXD → GPIO15 (RXD0)** *(Pi receives GNSS data)*
- **RXD → GPIO14 (TXD0)** *(Pi transmits to GNSS if needed)*

#### GNSS (UART)
| GNSS | Raspberry Pi |
|---|---|
| GNSS TX | Pi `GPIO15` (RXD0) |
| GNSS RX | Pi `GPIO14` (TXD0) |

#### Magnetometer (I2C)
| Magnetometer | Raspberry Pi |
|---|---|
| SDA | `GPIO2` |
| SCL | `GPIO3` |

---

## Power Architecture (Recommended)

### Battery (10A LiPo)
A “10A LiPo” description is often ambiguous. What matters:
- **Voltage** (e.g., 2S = 7.4V, 3S = 11.1V)
- **Capacity** (mAh)
- **C-rating** (current capability = Capacity(Ah) × C)

**Raspberry Pi 4 needs stable 5V** and is sensitive to brownouts.

### Recommended Layout
- **LiPo → DC-DC Buck Converter (5V, 5A+ recommended)** → Raspberry Pi 4
- **LiPo → Motor Driver (direct)** (depending on driver voltage range)
- Tie all grounds together:
  - Battery GND = Pi GND = Motor driver GND = sensor GND

> Do NOT power motors through the Pi.  
> Always use a dedicated buck converter for Pi 5V rail.

---

## Software Architecture

### What the code provides (Backbone)
- **Flask web server** with:
  - `/` dashboard: map + camera stream + sensor status
  - `/camera`: MJPEG stream
  - `/api/gps`: basic lat/lon (legacy endpoint kept for compatibility)
  - `/api/gps_detail`: fix quality, satellites, HDOP, altitude, speed/course (if available)
  - `/api/compass`: heading + calibration status
  - `/api/telemetry`: combined GPS + compass
  - `/api/compass/calibration/start`: begin calibration sampling
  - `/api/compass/calibration/stop`: compute + save offsets/scales
- **GPS reader thread**
  - Reads NMEA via `/dev/serial0`
  - Parses GGA/RMC
- **Magnetometer reader thread**
  - Auto-detects **QMC5883L (0x0D)** or **HMC5883L (0x1E)**
  - Provides heading, supports calibration
- **Gamepad driving loop**
  - Differential control (2 PWM)
  - Safety stop using MZ80 (front)

---

## Installation (Raspberry Pi OS)

### 1) Enable Interfaces
- Run:
```bash
sudo raspi-config
```
- Enable:

* I2C
* Serial / UART
* Disable login shell over serial
* Enable hardware serial port

- Reboot:
```bash
sudo reboot
```

## 2) System Packages

```bash
sudo apt update
sudo apt install -y \
  python3-pip python3-venv \
  python3-opencv \
  python3-serial \
  i2c-tools \
  python3-smbus \
  libatlas-base-dev
```

**!!! If python3-smbus is not enough on your OS image, use smbus2 via pip.**

## 3) Python Dependencies

- Create venv and install:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install flask pynmea2 pygame smbus2
```

## 4) Verify Hardware

- Check I2C devices:
```bash
i2cdetect -y 1
```

- You should see one of:

* 0x0D → QMC5883L
* 0x1E → HMC5883L

- Check GNSS UART stream:
```bash
sudo cat /dev/serial0
```
- You should see $GNGGA, $GNRMC, etc.

### Run

- Activate venv and start:
```bash
source .venv/bin/activate
python3 courier.py
```

- Open dashboard:
```bash
http://<raspberrypi-ip>:5000
```
- Endpoints:

Camera stream: http://<raspberrypi-ip>:5000/camera
Telemetry JSON: http://<raspberrypi-ip>:5000/api/telemetry


## GPS Usage Notes (Best Fix)

- To get stable GNSS:

* Go outdoors (open sky, minimal walls/metal)
* Wait for first fix (30–120 seconds typical)

- Aim for:

* Satellites: 8+ (more is better)
* HDOP: lower is better (e.g., < 1.5 good, < 1.0 great)

## Compass Calibration (Web UI)

- The dashboard provides:

* Start Calibration
* Stop & Save

- How to calibrate:

* Go outdoors (avoid large metal surfaces)
* Press Start Calibration
* Move the robot in a figure-8 (∞) motion for 30–60 seconds
* Press Stop & Save

- Calibration is stored locally:

* /home/pi/mag_calibration.json

**Note: This is a practical hard-iron + simple soft-iron correction.**

Tilt compensation requires an IMU and is not included in this build.

## Cellular Connectivity (USB 4G Modem)

- empty for now :)

## Safety (Read This)

* This robot drives real motors and can cause injury or damage.
* Always test at low speed in a controlled area.
* Use a physical kill switch.
* IR sensors can fail on black/reflective surfaces and under sunlight do not rely on them as the only safety layer.
* Raspberry Pi brownouts can cause unpredictable behavior ensure stable power.


### Roadmap to Full Autonomy (Next Milestones)

## Phase 1 — Reliable Backbone (this repo)

- ✅ Camera stream
- ✅ GNSS + compass telemetry
- ✅ Web map tracking + calibration flow
- ✅ Manual control + emergency stop

## Phase 2 — Perception (Pi 4 CPU optimized)

* Lightweight segmentation (drivable area / corridor / sidewalk)
* Export to TFLite INT8
* Run 8–15 FPS at low resolution

## Phase 3 — Planning & Control

- State machine:

* Manual / Auto / Emergency
* Visual lane/corridor centering controller
* GPS as “goal proximity” (single waypoint)

## Phase 4 — Robustness Upgrade (Recommended Hardware)

- To approach “street-level autonomy” safely:

* Wheel encoders (odometry)
* IMU (heading/tilt, sensor fusion)
* ToF/LiDAR (reliable obstacle distance)
* RTK GNSS (if you need accurate outdoor navigation)

## Repository Structure (Suggested)
```bash
.
├── courier.py
├── README.md
└── assets/
    ├── wiring/
    └── screenshots/
```

## License

MIT
