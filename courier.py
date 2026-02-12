import RPi.GPIO as GPIO
from flask import Flask, Response, jsonify, render_template_string, request
from picamera2 import Picamera2
import cv2
import threading
import time
import serial
import pynmea2
import os
import math
import json

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame


#Motorlar
PWM1_PIN = 18
DIR1_PIN = 23
PWM2_PIN = 13
DIR2_PIN = 6

#MZ80
MZ80_PIN = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(PWM1_PIN, GPIO.OUT)
GPIO.setup(DIR1_PIN, GPIO.OUT)
GPIO.setup(PWM2_PIN, GPIO.OUT)
GPIO.setup(DIR2_PIN, GPIO.OUT)

GPIO.setup(MZ80_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm1 = GPIO.PWM(PWM1_PIN, 1000)
pwm2 = GPIO.PWM(PWM2_PIN, 1000)

pwm1.start(0)
pwm2.start(0)

current_velocity = 0.0  
target_velocity = 0.0   
gear_direction = 1      
steering_angle = 0.0    


def apply_motor_power():
    global current_velocity, gear_direction, steering_angle

    
    base_speed = current_velocity

    
    if gear_direction == 1:  
        GPIO.output(DIR1_PIN, GPIO.LOW)
        GPIO.output(DIR2_PIN, GPIO.LOW)
    else:  
        GPIO.output(DIR1_PIN, GPIO.HIGH)
        GPIO.output(DIR2_PIN, GPIO.HIGH)

    
    speed_left = base_speed
    speed_right = base_speed

    if steering_angle > 0:  
        speed_right = base_speed * (1.0 - steering_angle)
    elif steering_angle < 0:  
        speed_left = base_speed * (1.0 - abs(steering_angle))

    
    speed_left = max(0, min(100, speed_left))
    speed_right = max(0, min(100, speed_right))

    pwm1.ChangeDutyCycle(speed_left)
    pwm2.ChangeDutyCycle(speed_right)

camera = Picamera2()
camera_config = camera.create_video_configuration(main={"size": (640, 480)})
camera.configure(camera_config)
camera.start()


def convert_to_degrees(raw_value):
    raw = float(raw_value)
    degrees = int(raw / 100)
    minutes = raw - (degrees * 100)
    return degrees + (minutes / 60.0)

try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    print("GPS bağlı (/dev/serial0).")
except Exception as e:
    ser = None
    print("GPS bulunamadı veya meşgul:", e)

current_lat = None
current_lon = None


gps_lock = threading.Lock()
gps_detail = {
    "fix": False,
    "fix_quality": None,
    "num_sats": None,
    "hdop": None,
    "alt_m": None,
    "speed_kmh": None,
    "course_deg": None,
    "last_update_ts": None,
    "raw_last_sentence": None,
}

def _safe_float(x):
    try:
        if x is None or x == "":
            return None
        return float(x)
    except Exception:
        return None

def _knots_to_kmh(kn):
    try:
        if kn is None:
            return None
        return float(kn) * 1.852
    except Exception:
        return None

def gps_reader():
    global current_lat, current_lon, gps_detail
    if ser is None:
        return

    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                time.sleep(0.05)
                continue

            
            if not (line.startswith("$GPGGA") or line.startswith("$GPRMC") or
                    line.startswith("$GNGGA") or line.startswith("$GNRMC")):
                continue

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            now_ts = time.time()
            with gps_lock:
                gps_detail["raw_last_sentence"] = line
                gps_detail["last_update_ts"] = now_ts

            
            if hasattr(msg, "lat") and hasattr(msg, "lon") and msg.lat and msg.lon:
                try:
                    lat = convert_to_degrees(msg.lat)
                    lon = convert_to_degrees(msg.lon)
                    if hasattr(msg, "lat_dir") and msg.lat_dir == "S":
                        lat = -lat
                    if hasattr(msg, "lon_dir") and msg.lon_dir == "W":
                        lon = -lon
                    current_lat = lat
                    current_lon = lon
                except Exception:
                    pass

            
            if msg.sentence_type == "GGA":
                fq = _safe_float(getattr(msg, "gps_qual", None))
                ns = _safe_float(getattr(msg, "num_sats", None))
                hd = _safe_float(getattr(msg, "horizontal_dil", None))
                alt = _safe_float(getattr(msg, "altitude", None))
                fix_ok = False
                try:
                    
                    fix_ok = fq is not None and fq > 0 and (current_lat is not None and current_lon is not None)
                except Exception:
                    fix_ok = False

                with gps_lock:
                    gps_detail["fix_quality"] = fq
                    gps_detail["num_sats"] = ns
                    gps_detail["hdop"] = hd
                    gps_detail["alt_m"] = alt
                    gps_detail["fix"] = bool(fix_ok)

            
            if msg.sentence_type == "RMC":
                sp_kn = _safe_float(getattr(msg, "spd_over_grnd", None))
                crs = _safe_float(getattr(msg, "true_course", None))
                with gps_lock:
                    gps_detail["speed_kmh"] = _knots_to_kmh(sp_kn)
                    gps_detail["course_deg"] = crs

        except Exception:
            time.sleep(0.1)

threading.Thread(target=gps_reader, daemon=True).start()


MAG_CAL_FILE = "/home/pi/mag_calibration.json"

mag_lock = threading.Lock()
mag_state = {
    "available": False,
    "chip": None,              
    "heading_deg": None,
    "raw": {"x": None, "y": None, "z": None},
    "calibrated": False,
    "cal": {
        "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "scale":  {"x": 1.0, "y": 1.0, "z": 1.0},
    },
    "calibration": {
        "running": False,
        "samples": 0,
        "min": {"x": None, "y": None, "z": None},
        "max": {"x": None, "y": None, "z": None},
        "started_ts": None,
    }
}

DECLINATION_DEG = 0.0

def load_mag_cal():
    global mag_state
    try:
        if os.path.exists(MAG_CAL_FILE):
            with open(MAG_CAL_FILE, "r") as f:
                data = json.load(f)
            if "offset" in data and "scale" in data:
                with mag_lock:
                    mag_state["cal"]["offset"] = data["offset"]
                    mag_state["cal"]["scale"] = data["scale"]
                    mag_state["calibrated"] = True
            print("[MAG] Kalibrasyon yüklendi:", MAG_CAL_FILE)
    except Exception as e:
        print("[MAG] Kalibrasyon yüklenemedi:", e)

def save_mag_cal(offset, scale):
    try:
        with open(MAG_CAL_FILE, "w") as f:
            json.dump({"offset": offset, "scale": scale}, f, indent=2)
        print("[MAG] Kalibrasyon kaydedildi:", MAG_CAL_FILE)
        return True
    except Exception as e:
        print("[MAG] Kalibrasyon kaydedilemedi:", e)
        return False


try:
    try:
        from smbus2 import SMBus
    except Exception:
        from smbus import SMBus
    i2c_bus = SMBus(1)
except Exception as e:
    i2c_bus = None
    print("[MAG] I2C bus açılamadı (bus=1):", e)

def i2c_probe(addr):
    if i2c_bus is None:
        return False
    try:
        i2c_bus.read_byte(addr)
        return True
    except Exception:
        return False


HMC_ADDR = 0x1E
HMC_REG_CONFIG_A = 0x00
HMC_REG_CONFIG_B = 0x01
HMC_REG_MODE     = 0x02
HMC_REG_DATA_X_MSB = 0x03

def hmc_init():
    i2c_bus.write_byte_data(HMC_ADDR, HMC_REG_CONFIG_A, 0x70)
    i2c_bus.write_byte_data(HMC_ADDR, HMC_REG_CONFIG_B, 0x20)
    i2c_bus.write_byte_data(HMC_ADDR, HMC_REG_MODE, 0x00)

def hmc_read_raw():
    data = i2c_bus.read_i2c_block_data(HMC_ADDR, HMC_REG_DATA_X_MSB, 6)
    x = (data[0] << 8) | data[1]
    z = (data[2] << 8) | data[3]
    y = (data[4] << 8) | data[5]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

QMC_ADDR = 0x0D
QMC_REG_CONTROL = 0x09
QMC_REG_SETRESET = 0x0B
QMC_REG_DATA = 0x00

def qmc_init():
    i2c_bus.write_byte_data(QMC_ADDR, QMC_REG_SETRESET, 0x01)
    i2c_bus.write_byte_data(QMC_ADDR, QMC_REG_CONTROL, 0xCD)

def qmc_read_raw():
    data = i2c_bus.read_i2c_block_data(QMC_ADDR, QMC_REG_DATA, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

def mag_apply_cal(x, y, z):
    with mag_lock:
        off = mag_state["cal"]["offset"]
        scl = mag_state["cal"]["scale"]
    
    x = (x - float(off["x"])) * float(scl["x"])
    y = (y - float(off["y"])) * float(scl["y"])
    z = (z - float(off["z"])) * float(scl["z"])
    return x, y, z

def mag_compute_heading_deg(x, y):
    
    heading = math.degrees(math.atan2(y, x))
    heading += DECLINATION_DEG
    heading = (heading + 360.0) % 360.0
    return heading

def mag_update_calibration_minmax(x, y, z):
    with mag_lock:
        cal = mag_state["calibration"]
        if not cal["running"]:
            return
        
        for axis, val in (("x", x), ("y", y), ("z", z)):
            if cal["min"][axis] is None or val < cal["min"][axis]:
                cal["min"][axis] = val
            if cal["max"][axis] is None or val > cal["max"][axis]:
                cal["max"][axis] = val
        cal["samples"] += 1

def mag_finalize_calibration():
    
    with mag_lock:
        cal = mag_state["calibration"]
        mn = cal["min"]
        mx = cal["max"]
        samples = cal["samples"]
        if samples < 200 or any(mn[a] is None or mx[a] is None for a in ("x", "y", "z")):
            return False, "Yetersiz örnek (en az ~200 örnek önerilir)."

        offset = {
            "x": (mx["x"] + mn["x"]) / 2.0,
            "y": (mx["y"] + mn["y"]) / 2.0,
            "z": (mx["z"] + mn["z"]) / 2.0,
        }
        
        rx = (mx["x"] - mn["x"]) / 2.0
        ry = (mx["y"] - mn["y"]) / 2.0
        rz = (mx["z"] - mn["z"]) / 2.0
        avg = (rx + ry + rz) / 3.0 if (rx > 0 and ry > 0 and rz > 0) else None
        if avg is None or avg == 0:
            return False, "Kalibrasyon başarısız (ölçüm aralığı sorunlu)."

        scale = {
            "x": avg / rx if rx != 0 else 1.0,
            "y": avg / ry if ry != 0 else 1.0,
            "z": avg / rz if rz != 0 else 1.0,
        }

        
        mag_state["cal"]["offset"] = offset
        mag_state["cal"]["scale"] = scale
        mag_state["calibrated"] = True

        
        mag_state["calibration"]["running"] = False

    ok = save_mag_cal(offset, scale)
    if not ok:
        return False, "Kalibrasyon hesaplandı ama dosyaya kaydedilemedi."
    return True, "Kalibrasyon tamamlandı ve kaydedildi."

def mag_reader():
    global mag_state
    if i2c_bus is None:
        return

    
    chip = None
    try:
        if i2c_probe(QMC_ADDR):
            chip = "QMC5883L"
            qmc_init()
        elif i2c_probe(HMC_ADDR):
            chip = "HMC5883L"
            hmc_init()
    except Exception as e:
        print("[MAG] Algılama/init hatası:", e)
        chip = None

    if chip is None:
        with mag_lock:
            mag_state["available"] = False
            mag_state["chip"] = None
        print("[MAG] Manyetometre bulunamadı (0x0D veya 0x1E).")
        return

    with mag_lock:
        mag_state["available"] = True
        mag_state["chip"] = chip

    print(f"[MAG] Manyetometre aktif: {chip}")
    load_mag_cal()

    while True:
        try:
            if chip == "QMC5883L":
                x, y, z = qmc_read_raw()
            else:
                x, y, z = hmc_read_raw()

            
            mag_update_calibration_minmax(x, y, z)

            
            with mag_lock:
                calibrated = mag_state["calibrated"]

            if calibrated:
                cx, cy, cz = mag_apply_cal(x, y, z)
            else:
                cx, cy, cz = x, y, z

            heading = mag_compute_heading_deg(cx, cy)

            with mag_lock:
                mag_state["raw"] = {"x": x, "y": y, "z": z}
                mag_state["heading_deg"] = heading

            time.sleep(0.05)  
        except Exception:
            time.sleep(0.1)

threading.Thread(target=mag_reader, daemon=True).start()


app = Flask(__name__)

def generate_frames():
    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ret, buffer = cv2.imencode(".jpg", frame)
        if ret:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")
        time.sleep(0.03)

@app.route('/camera')
def camera_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/api/gps")
def gps_api():
    return jsonify({"lat": current_lat, "lon": current_lon})


@app.route("/api/gps_detail")
def gps_detail_api():
    with gps_lock:
        d = dict(gps_detail)
    return jsonify({
        "lat": current_lat,
        "lon": current_lon,
        **d
    })


@app.route("/api/compass")
def compass_api():
    with mag_lock:
        d = {
            "available": mag_state["available"],
            "chip": mag_state["chip"],
            "heading_deg": mag_state["heading_deg"],
            "raw": mag_state["raw"],
            "calibrated": mag_state["calibrated"],
            "calibration": mag_state["calibration"],
        }
    return jsonify(d)


@app.route("/api/telemetry")
def telemetry_api():
    with gps_lock:
        gd = dict(gps_detail)
    with mag_lock:
        md = {
            "available": mag_state["available"],
            "chip": mag_state["chip"],
            "heading_deg": mag_state["heading_deg"],
            "calibrated": mag_state["calibrated"],
            "calibration": mag_state["calibration"],
        }
    return jsonify({
        "gps": {"lat": current_lat, "lon": current_lon, **gd},
        "compass": md
    })


@app.route("/api/compass/calibration/start", methods=["POST"])
def compass_cal_start():
    with mag_lock:
        if not mag_state["available"]:
            return jsonify({"ok": False, "msg": "Manyetometre bulunamadı."}), 400
        mag_state["calibration"] = {
            "running": True,
            "samples": 0,
            "min": {"x": None, "y": None, "z": None},
            "max": {"x": None, "y": None, "z": None},
            "started_ts": time.time(),
        }
    return jsonify({"ok": True, "msg": "Kalibrasyon başladı. Robotu açık alanda 30-60 sn '∞/figure-8' hareketiyle döndür."})

@app.route("/api/compass/calibration/stop", methods=["POST"])
def compass_cal_stop():
    ok, msg = mag_finalize_calibration()
    code = 200 if ok else 400
    return jsonify({"ok": ok, "msg": msg}), code

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Kontrol - GPS + Pusula Live</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
    <style>
        body { margin:0; padding:0; font-family: Arial; background: #222; color: white; }
        #map { width: 100%; height: 50vh; }
        #cam { width: 100%; height: auto; display:block; }
        .info { padding: 10px; background: #333; }
        .row { display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
        .card { background:#2b2b2b; padding:10px; border-radius:10px; min-width:220px; }
        .btn { background:#444; color:white; border:none; padding:10px 12px; border-radius:10px; cursor:pointer; }
        .btn:hover { background:#555; }
        .badge { display:inline-block; padding:4px 8px; border-radius:999px; background:#444; }
        .good { background:#1f6f3f; }
        .warn { background:#7a5a11; }
        .bad  { background:#7a1f1f; }
        .small { font-size: 12px; opacity: 0.9; }
        
        .heading-icon {
            width: 0; height: 0;
            border-left: 10px solid transparent;
            border-right: 10px solid transparent;
            border-bottom: 22px solid #00d1ff;
            transform-origin: 50% 70%;
            filter: drop-shadow(0 0 6px rgba(0,209,255,0.35));
        }
        .heading-wrap {
            width: 26px; height: 26px;
            display:flex; align-items:center; justify-content:center;
        }
    </style>
</head>
<body>
    <h2 style="text-align:center; margin:12px 0;">Robot Kontrol Merkezi</h2>

    <div class="info">
        <div class="row">
            <div class="card">
                <div><b>GPS Kurulum / Yönlendirme</b></div>
                <div class="small">
                    1) Açık alana çık (gökyüzü görünür olsun).<br>
                    2) İlk fix için 30-120 sn bekle.<br>
                    3) Uydu sayısı & HDOP iyileştikçe konum stabil olur.
                </div>
                <div style="margin-top:8px">
                    <span id="gpsBadge" class="badge bad">GPS: NO FIX</span>
                </div>
            </div>

            <div class="card">
                <div><b>Pusula / Kalibrasyon</b></div>
                <div class="small">
                    İlk kullanımda kalibrasyon önerilir:<br>
                    Açık alanda robotu 30-60 sn “∞/figure-8” şeklinde döndür.
                </div>
                <div style="margin-top:8px" class="row">
                    <button class="btn" onclick="startCal()">Kalibrasyonu Başlat</button>
                    <button class="btn" onclick="stopCal()">Durdur & Kaydet</button>
                </div>
                <div style="margin-top:8px">
                    <span id="magBadge" class="badge warn">PUSULA: ...</span>
                    <span id="calBadge" class="badge warn">CAL: ...</span>
                </div>
            </div>

            <div class="card">
                <div><b>Canlı Telemetri</b></div>
                <div class="small" id="telemetryText">Yükleniyor...</div>
            </div>
        </div>
    </div>

    <img id="cam" src="/camera">
    <div id="map"></div>

    <script>
        
        var map = L.map('map').setView([0, 0], 17);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20 }).addTo(map);

        
        var headingDiv = document.createElement("div");
        headingDiv.className = "heading-wrap";
        var arrow = document.createElement("div");
        arrow.className = "heading-icon";
        headingDiv.appendChild(arrow);

        var headingIcon = L.divIcon({
            html: headingDiv,
            className: "",
            iconSize: [26,26],
            iconAnchor: [13,13]
        });

        var marker = L.marker([0,0], {icon: headingIcon}).addTo(map);
        var path = L.polyline([], {weight:3}).addTo(map);

        var first = true;
        var lastLat = null, lastLon = null;
        var lastAddTs = 0;

        function setArrowHeading(deg) {
            if (deg === null || deg === undefined) return;
            arrow.style.transform = "rotate(" + deg + "deg)";
        }

        function badge(el, text, kind) {
            el.textContent = text;
            el.classList.remove("good","warn","bad");
            el.classList.add(kind);
        }

        function updateUI(data) {
            var gps = data.gps;
            var comp = data.compass;

            
            var gpsB = document.getElementById("gpsBadge");
            if (gps && gps.fix) badge(gpsB, "GPS: FIX", "good");
            else badge(gpsB, "GPS: NO FIX", "bad");

            
            var magB = document.getElementById("magBadge");
            if (comp && comp.available) badge(magB, "PUSULA: " + (comp.chip || "OK"), "good");
            else badge(magB, "PUSULA: YOK", "bad");

            
            var calB = document.getElementById("calBadge");
            if (comp && comp.calibration && comp.calibration.running) {
                badge(calB, "CAL: RUNNING (" + comp.calibration.samples + ")", "warn");
            } else {
                badge(calB, "CAL: " + ((comp && comp.calibrated) ? "OK" : "NOT CAL"), (comp && comp.calibrated) ? "good" : "warn");
            }

            
            var t = [];
            if (gps) {
                t.push("Lat/Lon: " + (gps.lat ?? "-") + ", " + (gps.lon ?? "-"));
                t.push("Sats: " + (gps.num_sats ?? "-") + " | HDOP: " + (gps.hdop ?? "-"));
                t.push("Alt: " + (gps.alt_m ?? "-") + " m | Speed: " + (gps.speed_kmh ? gps.speed_kmh.toFixed(1) : "-") + " km/h");
                t.push("Course(GNSS): " + (gps.course_deg ?? "-") + "°");
            }
            if (comp) {
                t.push("Heading(MAG): " + (comp.heading_deg ? comp.heading_deg.toFixed(1) : "-") + "°");
            }
            document.getElementById("telemetryText").innerHTML = t.join("<br>");
        }

        function updateMap(gps, comp) {
            if (!gps || gps.lat === null || gps.lon === null) return;

            var lat = parseFloat(gps.lat);
            var lon = parseFloat(gps.lon);

            marker.setLatLng([lat, lon]);

            
            var heading = null;
            if (comp && comp.heading_deg !== null && comp.heading_deg !== undefined) heading = comp.heading_deg;
            else if (gps.course_deg !== null && gps.course_deg !== undefined) heading = gps.course_deg;

            setArrowHeading(heading);

            if (first) {
                map.setView([lat, lon], 18);
                first = false;
            }

            
            var now = Date.now();
            if (lastLat === null || lastLon === null) {
                path.addLatLng([lat, lon]);
                lastLat = lat; lastLon = lon; lastAddTs = now;
                return;
            }

            var dLat = (lat - lastLat);
            var dLon = (lon - lastLon);
            var approx = Math.sqrt(dLat*dLat + dLon*dLon); 
            if (approx > 0.00001 || (now - lastAddTs) > 1000) { 
                path.addLatLng([lat, lon]);
                lastLat = lat; lastLon = lon; lastAddTs = now;
            }
        }

        async function fetchTelemetry() {
            try {
                const res = await fetch('/api/telemetry');
                const data = await res.json();
                updateUI(data);
                updateMap(data.gps, data.compass);
            } catch(e) {}
        }

        async function startCal() {
            try {
                const res = await fetch('/api/compass/calibration/start', {method:'POST'});
                const data = await res.json();
                alert(data.msg || "OK");
            } catch(e) { alert("Başlatma hatası"); }
        }

        async function stopCal() {
            try {
                const res = await fetch('/api/compass/calibration/stop', {method:'POST'});
                const data = await res.json();
                alert(data.msg || "OK");
            } catch(e) { alert("Durdurma hatası"); }
        }

        setInterval(fetchTelemetry, 500);
        fetchTelemetry();
    </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

def start_flask():
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


def gamepad_physics_loop():
    global current_velocity, target_velocity, gear_direction, steering_angle

    
    ACCELERATION_RATE = 2.0
    COASTING_RATE = 1.0
    BRAKING_RATE = 4.0
    MAX_SPEED = 100.0

    pygame.init()
    pygame.joystick.init()

    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"\n[INFO] Gamepad: {joystick.get_name()}")
    else:
        print("\n[UYARI] Gamepad yok! Sistem bekliyor...")

    last_r1_state = 0
    print("Sistem Hazır. MZ80 Engel Koruması Aktif.")

    try:
        while True:
            
            pygame.event.pump()

            throttle_input = 0.0
            brake_input = 0.0
            steering_input = 0.0
            r1_pressed = 0

            if joystick:
                
                raw_r2 = joystick.get_axis(5)  
                throttle_input = (raw_r2 + 1) / 2.0

                raw_l2 = joystick.get_axis(2)  
                brake_input = (raw_l2 + 1) / 2.0

                steering_input = joystick.get_axis(0)  
                r1_pressed = joystick.get_button(5)    

            
            if r1_pressed and not last_r1_state:
                gear_direction *= -1
                mode = "İLERİ" if gear_direction == 1 else "GERİ"
                print(f"Vites: {mode}")
            last_r1_state = r1_pressed

            
            obstacle_detected = (GPIO.input(MZ80_PIN) == 0)

            
            if obstacle_detected and gear_direction == 1:
                target_velocity = 0
                current_velocity = 0  
            else:
                
                target_velocity = throttle_input * MAX_SPEED

                if brake_input > 0.1:
                    drop_amount = BRAKING_RATE * (0.5 + brake_input)
                    if current_velocity > 0:
                        current_velocity -= drop_amount
                    if current_velocity < 0:
                        current_velocity = 0
                else:
                    if current_velocity < target_velocity:
                        current_velocity += ACCELERATION_RATE
                        if current_velocity > target_velocity:
                            current_velocity = target_velocity
                    elif current_velocity > target_velocity:
                        current_velocity -= COASTING_RATE
                        if current_velocity < target_velocity:
                            current_velocity = target_velocity

            current_velocity = max(0.0, current_velocity)

            
            if abs(steering_input) < 0.1:
                steering_angle = 0.0
            else:
                steering_angle = steering_input

            
            apply_motor_power()

            time.sleep(0.05)  

    except KeyboardInterrupt:
        print("Çıkış yapılıyor...")
    except Exception as e:
        print(f"Hata oluştu: {e}")


try:
    threading.Thread(target=start_flask, daemon=True).start()
    gamepad_physics_loop()

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    try:
        camera.stop()
    except:
        pass
    try:
        if ser:
            ser.close()
    except:
        pass
    try:
        if i2c_bus:
            i2c_bus.close()
    except:
        pass
    print("Tüm sistemler kapatıldı.")
