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
import sys
import tty
import termios
import select

os.environ["SDL_VIDEODRIVER"] = "dummy"

PWM1_PIN = 18  # WHITE
DIR1_PIN = 23
PWM2_PIN = 13  # WHITE
DIR2_PIN = 6

MZ80_FRONT_PIN      = 17
MZ80_LEFT_FRONT_PIN = 16
MZ80_RIGHT_FRONT_PIN = 22
MZ80_RIGHT_REAR_PIN  = 26
MZ80_LEFT_REAR_PIN   = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM1_PIN,  GPIO.OUT)
GPIO.setup(DIR1_PIN,  GPIO.OUT)
GPIO.setup(PWM2_PIN,  GPIO.OUT)
GPIO.setup(DIR2_PIN,  GPIO.OUT)
GPIO.setup(MZ80_FRONT_PIN,       GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MZ80_LEFT_FRONT_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MZ80_RIGHT_FRONT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MZ80_RIGHT_REAR_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MZ80_LEFT_REAR_PIN,   GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm1 = GPIO.PWM(PWM1_PIN, 1000)
pwm2 = GPIO.PWM(PWM2_PIN, 1000)
pwm1.start(0)
pwm2.start(0)

current_velocity = 0.0
target_velocity  = 0.0
gear_direction   = 1     
steering_angle   = 0.0  


camera = Picamera2()
camera_config = camera.create_video_configuration(main={"size": (640, 480)})
camera.configure(camera_config)
camera.start()


def convert_to_degrees(raw_value):
    raw     = float(raw_value)
    degrees = int(raw / 100)
    minutes = raw - (degrees * 100)
    return degrees + (minutes / 60.0)

current_lat = None
current_lon = None
gps_lock    = threading.Lock()
gps_detail  = {
    "fix": False, "fix_quality": None, "gsa_fix_type": None,
    "rmc_status": None, "num_sats": None, "hdop": None,
    "pdop": None, "vdop": None, "alt_m": None, "speed_kmh": None,
    "course_deg": None, "last_update_ts": None, "last_fix_ts": None,
    "raw_last_sentence": None, "port": None, "baud": None,
    "nmea_seen": False, "errors": 0,
}

def _safe_float(x):
    try:
        return None if (x is None or x == "") else float(x)
    except Exception:
        return None

def _knots_to_kmh(kn):
    try:
        return None if (kn is None or kn == "") else float(kn) * 1.852
    except Exception:
        return None

def _now():
    return time.time()

def _update_fix_fields():
    """gps_lock dışarıda alınmışken çağrılır — tekrar lock almaz (deadlock önleme)."""
    global current_lat, current_lon
    lat_ok  = current_lat is not None and current_lon is not None
    rmc_ok  = (gps_detail.get("rmc_status") == "A")
    gga_ok  = (gps_detail.get("fix_quality") is not None and gps_detail.get("fix_quality") > 0)
    gsa_ok  = (gps_detail.get("gsa_fix_type") is not None and gps_detail.get("gsa_fix_type") >= 2)
    fix_ok  = lat_ok and (rmc_ok or gga_ok or gsa_ok)
    gps_detail["fix"] = bool(fix_ok)
    if fix_ok:
        gps_detail["last_fix_ts"] = _now()

def _open_serial_any():
    ports = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]
    bauds = [9600, 38400, 57600, 115200]
    for p in ports:
        for b in bauds:
            try:
                s = serial.Serial(p, b, timeout=1)
                t0 = time.time()
                saw_dollar = False
                while time.time() - t0 < 1.2:
                    line = s.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("$") and len(line) > 6:
                        saw_dollar = True
                        break
                if saw_dollar:
                    with gps_lock:
                        gps_detail["port"] = p
                        gps_detail["baud"] = b
                        gps_detail["nmea_seen"] = True
                    print(f"[GPS] NMEA bulundu: {p} @ {b}")
                    return s
                s.close()
            except Exception:
                continue
    return None

ser = _open_serial_any()
if ser is None:
    print("[GPS] Port/baud otomatik bulunamadı.")
    with gps_lock:
        gps_detail["errors"] += 1

def gps_reader():
    global current_lat, current_lon, ser
    if ser is None:
        while True:
            time.sleep(2.0)
            s = _open_serial_any()
            if s is not None:
                ser = s
                break
    while True:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line or not line.startswith("$"):
                time.sleep(0.02)
                continue
            now_ts = _now()
            with gps_lock:
                gps_detail["raw_last_sentence"] = line
                gps_detail["last_update_ts"]    = now_ts
                gps_detail["nmea_seen"]          = True
            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue
            if hasattr(msg, "lat") and hasattr(msg, "lon") and msg.lat and msg.lon:
                try:
                    lat = convert_to_degrees(msg.lat)
                    lon = convert_to_degrees(msg.lon)
                    if hasattr(msg, "lat_dir") and msg.lat_dir == "S": lat = -lat
                    if hasattr(msg, "lon_dir") and msg.lon_dir == "W": lon = -lon
                    current_lat = lat
                    current_lon = lon
                except Exception:
                    pass
            if getattr(msg, "sentence_type", None) == "GGA":
                with gps_lock:
                    gps_detail["fix_quality"] = _safe_float(getattr(msg, "gps_qual", None))
                    gps_detail["num_sats"]    = _safe_float(getattr(msg, "num_sats", None))
                    gps_detail["hdop"]        = _safe_float(getattr(msg, "horizontal_dil", None))
                    gps_detail["alt_m"]       = _safe_float(getattr(msg, "altitude", None))
                    _update_fix_fields()
            if getattr(msg, "sentence_type", None) == "RMC":
                with gps_lock:
                    gps_detail["rmc_status"] = getattr(msg, "status", None)
                    gps_detail["speed_kmh"]  = _knots_to_kmh(_safe_float(getattr(msg, "spd_over_grnd", None)))
                    gps_detail["course_deg"] = _safe_float(getattr(msg, "true_course", None))
                    _update_fix_fields()
            if getattr(msg, "sentence_type", None) == "GSA":
                with gps_lock:
                    gps_detail["gsa_fix_type"] = _safe_float(getattr(msg, "mode_fix_type", None))
                    gps_detail["pdop"]          = _safe_float(getattr(msg, "pdop", None))
                    gps_detail["vdop"]          = _safe_float(getattr(msg, "vdop", None))
                    hd = _safe_float(getattr(msg, "hdop", None))
                    if hd is not None:
                        gps_detail["hdop"] = hd
                    _update_fix_fields()
        except Exception:
            with gps_lock:
                gps_detail["errors"] += 1
            time.sleep(0.1)

threading.Thread(target=gps_reader, daemon=True).start()

MAG_CAL_FILE   = "/home/pi/mag_calibration.json"
DECLINATION_DEG = 0.0
mag_lock = threading.Lock()
mag_state = {
    "available": False, "chip": None, "heading_deg": None,
    "raw": {"x": None, "y": None, "z": None},
    "calibrated": False,
    "cal": {"offset": {"x": 0.0, "y": 0.0, "z": 0.0},
            "scale":  {"x": 1.0, "y": 1.0, "z": 1.0}},
    "calibration": {
        "running": False, "samples": 0,
        "min": {"x": None, "y": None, "z": None},
        "max": {"x": None, "y": None, "z": None},
        "started_ts": None,
    }
}

def load_mag_cal():
    try:
        if os.path.exists(MAG_CAL_FILE):
            with open(MAG_CAL_FILE, "r") as f:
                data = json.load(f)
            if "offset" in data and "scale" in data:
                with mag_lock:
                    mag_state["cal"]["offset"] = data["offset"]
                    mag_state["cal"]["scale"]  = data["scale"]
                    mag_state["calibrated"]    = True
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
    print("[MAG] I2C bus açılamadı:", e)

def i2c_probe(addr):
    if i2c_bus is None:
        return False
    try:
        i2c_bus.read_byte(addr)
        return True
    except Exception:
        return False

HMC_ADDR = 0x1E
QMC_ADDR = 0x0E
IST_ADDR = 0x0E

def hmc_init():
    i2c_bus.write_byte_data(HMC_ADDR, 0x00, 0x70)
    i2c_bus.write_byte_data(HMC_ADDR, 0x01, 0x20)
    i2c_bus.write_byte_data(HMC_ADDR, 0x02, 0x00)

def hmc_read_raw():
    data = i2c_bus.read_i2c_block_data(HMC_ADDR, 0x03, 6)
    x = (data[0] << 8) | data[1]; z = (data[2] << 8) | data[3]; y = (data[4] << 8) | data[5]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

def qmc_init():
    i2c_bus.write_byte_data(QMC_ADDR, 0x0B, 0x01)
    i2c_bus.write_byte_data(QMC_ADDR, 0x09, 0xCD)

def qmc_read_raw():
    data = i2c_bus.read_i2c_block_data(QMC_ADDR, 0x00, 6)
    x = (data[1] << 8) | data[0]; y = (data[3] << 8) | data[2]; z = (data[5] << 8) | data[4]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

def ist8310_init():
    pass

def ist8310_read_raw():
    i2c_bus.write_byte_data(IST_ADDR, 0x0A, 0x01)
    time.sleep(0.01)
    data = i2c_bus.read_i2c_block_data(IST_ADDR, 0x03, 6)
    x = (data[1] << 8) | data[0]; y = (data[3] << 8) | data[2]; z = (data[5] << 8) | data[4]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

def mag_apply_cal(x, y, z):
    with mag_lock:
        off = mag_state["cal"]["offset"]
        scl = mag_state["cal"]["scale"]
    return ((x - float(off["x"])) * float(scl["x"]),
            (y - float(off["y"])) * float(scl["y"]),
            (z - float(off["z"])) * float(scl["z"]))

def mag_compute_heading_deg(x, y):
    h = math.degrees(math.atan2(y, x)) + DECLINATION_DEG
    return (h + 360.0) % 360.0

def mag_update_calibration_minmax(x, y, z):
    with mag_lock:
        cal = mag_state["calibration"]
        if not cal["running"]:
            return
        for axis, val in (("x", x), ("y", y), ("z", z)):
            if cal["min"][axis] is None or val < cal["min"][axis]: cal["min"][axis] = val
            if cal["max"][axis] is None or val > cal["max"][axis]: cal["max"][axis] = val
        cal["samples"] += 1

def mag_finalize_calibration():
    with mag_lock:
        cal = mag_state["calibration"]
        mn  = cal["min"]; mx = cal["max"]
        if cal["samples"] < 200 or any(mn[a] is None or mx[a] is None for a in ("x","y","z")):
            return False, "Yetersiz örnek (en az ~200 önerilir)."
        offset = {a: (mx[a] + mn[a]) / 2.0 for a in ("x","y","z")}
        rx = (mx["x"] - mn["x"]) / 2.0
        ry = (mx["y"] - mn["y"]) / 2.0
        rz = (mx["z"] - mn["z"]) / 2.0
        avg = (rx + ry + rz) / 3.0 if (rx > 0 and ry > 0 and rz > 0) else None
        if not avg:
            return False, "Kalibrasyon başarısız."
        scale = {a: avg / r if r != 0 else 1.0 for a, r in (("x",rx),("y",ry),("z",rz))}
        mag_state["cal"]["offset"]  = offset
        mag_state["cal"]["scale"]   = scale
        mag_state["calibrated"]     = True
        mag_state["calibration"]["running"] = False
    return save_mag_cal(offset, scale), "Kalibrasyon tamamlandı."

def mag_reader():
    if i2c_bus is None:
        return
    chip = None
    try:
        if i2c_probe(0x0E):
            try:
                who = i2c_bus.read_byte_data(0x0E, 0x00)
                if who == 0x10:
                    chip = "IST8310"; ist8310_init()
                else:
                    chip = "QMC5883L"; qmc_init()
            except Exception:
                chip = "QMC5883L"; qmc_init()
        elif i2c_probe(HMC_ADDR):
            chip = "HMC5883L"; hmc_init()
    except Exception as e:
        print("[MAG] Algılama hatası:", e)
    if chip is None:
        with mag_lock:
            mag_state["available"] = False
        print("[MAG] Manyetometre bulunamadı.")
        return
    with mag_lock:
        mag_state["available"] = True
        mag_state["chip"]      = chip
    print(f"[MAG] Aktif: {chip}")
    load_mag_cal()
    while True:
        try:
            if chip == "IST8310":  x,y,z = ist8310_read_raw()
            elif chip == "QMC5883L": x,y,z = qmc_read_raw()
            else:                    x,y,z = hmc_read_raw()
            mag_update_calibration_minmax(x, y, z)
            with mag_lock:
                cal = mag_state["calibrated"]
            cx,cy,cz = mag_apply_cal(x,y,z) if cal else (x,y,z)
            heading  = mag_compute_heading_deg(cx, cy)
            with mag_lock:
                mag_state["raw"]         = {"x":x,"y":y,"z":z}
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

@app.route("/camera")
def camera_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/gps")
def gps_api():
    return jsonify({"lat": current_lat, "lon": current_lon})

@app.route("/api/gps_detail")
def gps_detail_api():
    with gps_lock:
        d = dict(gps_detail)
    return jsonify({"lat": current_lat, "lon": current_lon, **d})

@app.route("/api/compass")
def compass_api():
    with mag_lock:
        d = {k: mag_state[k] for k in ("available","chip","heading_deg","raw","calibrated","calibration")}
    return jsonify(d)

@app.route("/api/telemetry")
def telemetry_api():
    with gps_lock:
        gd = dict(gps_detail)
    with mag_lock:
        md = {k: mag_state[k] for k in ("available","chip","heading_deg","calibrated","calibration")}
    return jsonify({"gps": {"lat": current_lat, "lon": current_lon, **gd}, "compass": md})

@app.route("/api/compass/calibration/start", methods=["POST"])
def compass_cal_start():
    with mag_lock:
        if not mag_state["available"]:
            return jsonify({"ok": False, "msg": "Manyetometre bulunamadı."}), 400
        mag_state["calibration"] = {
            "running": True, "samples": 0,
            "min": {"x":None,"y":None,"z":None},
            "max": {"x":None,"y":None,"z":None},
            "started_ts": time.time(),
        }
    return jsonify({"ok": True, "msg": "Kalibrasyon başladı."})

@app.route("/api/compass/calibration/stop", methods=["POST"])
def compass_cal_stop():
    ok, msg = mag_finalize_calibration()
    return jsonify({"ok": ok, "msg": msg}), (200 if ok else 400)

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
        .card { background:#2b2b2b; padding:10px; border-radius:10px; min-width:240px; }
        .btn { background:#444; color:white; border:none; padding:10px 12px; border-radius:10px; cursor:pointer; }
        .btn:hover { background:#555; }
        .badge { display:inline-block; padding:4px 8px; border-radius:999px; background:#444; }
        .good { background:#1f6f3f; } .warn { background:#7a5a11; } .bad { background:#7a1f1f; }
        .small { font-size: 12px; opacity: 0.9; }
        .mono { font-family: monospace; }
        .heading-icon { width:0; height:0; border-left:10px solid transparent; border-right:10px solid transparent; border-bottom:22px solid #00d1ff; transform-origin:50% 70%; filter:drop-shadow(0 0 6px rgba(0,209,255,0.35)); }
        .heading-wrap { width:26px; height:26px; display:flex; align-items:center; justify-content:center; }
    </style>
</head>
<body>
    <h2 style="text-align:center; margin:12px 0;">Robot Kontrol Merkezi</h2>
    <div class="info">
        <div class="row">
            <div class="card">
                <div><b>GPS Durum</b></div>
                <div style="margin-top:8px">
                    <span id="gpsBadge" class="badge bad">GPS: NO FIX</span>
                    <span id="uartBadge" class="badge warn">UART: ...</span>
                </div>
                <div class="small mono" id="nmeaLine">NMEA: -</div>
            </div>
            <div class="card">
                <div><b>Pusula / Kalibrasyon</b></div>
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
        var map = L.map('map').setView([0,0], 17);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:20}).addTo(map);
        var headingDiv = document.createElement("div"); headingDiv.className="heading-wrap";
        var arrow = document.createElement("div"); arrow.className="heading-icon";
        headingDiv.appendChild(arrow);
        var headingIcon = L.divIcon({html:headingDiv, className:"", iconSize:[26,26], iconAnchor:[13,13]});
        var marker = L.marker([0,0], {icon:headingIcon}).addTo(map);
        var path = L.polyline([], {weight:3}).addTo(map);
        var first=true, lastLat=null, lastLon=null, lastAddTs=0;
        function setArrowHeading(deg){if(deg===null||deg===undefined)return; arrow.style.transform="rotate("+deg+"deg)";}
        function badge(el,text,kind){el.textContent=text; el.classList.remove("good","warn","bad"); el.classList.add(kind);}
        function updateUI(data){
            var gps=data.gps, comp=data.compass;
            badge(document.getElementById("gpsBadge"), gps&&gps.fix?"GPS: FIX":"GPS: NO FIX", gps&&gps.fix?"good":"bad");
            badge(document.getElementById("uartBadge"), gps&&gps.port?"UART: "+gps.port+" @ "+gps.baud:"UART: searching...", gps&&gps.port?"good":"warn");
            document.getElementById("nmeaLine").textContent="NMEA: "+(gps&&gps.raw_last_sentence?gps.raw_last_sentence:"-");
            badge(document.getElementById("magBadge"), comp&&comp.available?"PUSULA: "+(comp.chip||"OK"):"PUSULA: YOK", comp&&comp.available?"good":"bad");
            var calB=document.getElementById("calBadge");
            if(comp&&comp.calibration&&comp.calibration.running) badge(calB,"CAL: RUNNING ("+comp.calibration.samples+")","warn");
            else badge(calB,"CAL: "+((comp&&comp.calibrated)?"OK":"NOT CAL"),(comp&&comp.calibrated)?"good":"warn");
            var t=[];
            if(gps){
                t.push("Lat/Lon: "+(gps.lat??"-")+", "+(gps.lon??"-"));
                t.push("Sats: "+(gps.num_sats??"-")+" | HDOP: "+(gps.hdop??"-"));
                t.push("Alt: "+(gps.alt_m??"-")+" m | Speed: "+(gps.speed_kmh?gps.speed_kmh.toFixed(1):"-")+" km/h");
            }
            if(comp) t.push("Heading: "+(comp.heading_deg?comp.heading_deg.toFixed(1):"-")+"°");
            document.getElementById("telemetryText").innerHTML=t.join("<br>");
        }
        function updateMap(gps,comp){
            if(!gps||gps.lat===null||gps.lon===null)return;
            var lat=parseFloat(gps.lat), lon=parseFloat(gps.lon);
            marker.setLatLng([lat,lon]);
            var heading=(comp&&comp.heading_deg!=null)?comp.heading_deg:(gps.course_deg??null);
            setArrowHeading(heading);
            if(first){map.setView([lat,lon],18); first=false;}
            var now=Date.now();
            if(lastLat===null){path.addLatLng([lat,lon]); lastLat=lat; lastLon=lon; lastAddTs=now; return;}
            var d=Math.sqrt(Math.pow(lat-lastLat,2)+Math.pow(lon-lastLon,2));
            if(d>0.00001||(now-lastAddTs)>1000){path.addLatLng([lat,lon]); lastLat=lat; lastLon=lon; lastAddTs=now;}
        }
        async function fetchTelemetry(){
            try{const r=await fetch('/api/telemetry'); const d=await r.json(); updateUI(d); updateMap(d.gps,d.compass);}catch(e){}
        }
        async function postAndAlert(url){
            try{const r=await fetch(url,{method:'POST'}); let d=null; try{d=await r.json();}catch(_){} alert((d&&d.msg)?d.msg:"HTTP "+r.status);}
            catch(e){alert("Hata: "+e);}
        }
        function startCal(){postAndAlert('/api/compass/calibration/start');}
        function stopCal(){postAndAlert('/api/compass/calibration/stop');}
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

obstacle_detected = False

def apply_motor_power():
    global current_velocity, gear_direction, steering_angle, obstacle_detected

    base_speed = current_velocity

    if gear_direction == 1 and obstacle_detected:
        GPIO.output(DIR1_PIN, GPIO.LOW)   
        GPIO.output(DIR2_PIN, GPIO.HIGH)  
        base_speed = 0
    else:
        if gear_direction == 1:       
            GPIO.output(DIR1_PIN, GPIO.HIGH) 
            GPIO.output(DIR2_PIN, GPIO.LOW)  
        else:                           
            GPIO.output(DIR1_PIN, GPIO.LOW)   
            GPIO.output(DIR2_PIN, GPIO.HIGH)  

    speed_left  = base_speed
    speed_right = base_speed

    if steering_angle > 0:
        speed_right = base_speed * (1.0 - steering_angle)
    elif steering_angle < 0:
        speed_left  = base_speed * (1.0 - abs(steering_angle))

    pwm1.ChangeDutyCycle(max(0, min(100, speed_left)))
    pwm2.ChangeDutyCycle(max(0, min(100, speed_right)))


HELP_TEXT = """
╔══════════════════════════════════════════╗
║         ROBOT WASD KONTROL               ║
╠══════════════════════════════════════════╣
║  W          → Gaz (hızlan)               ║
║  S          → Frenleme / Dur             ║
║  R          → Vites değiştir (İleri/Geri)║
║  SPACE      → Acil dur (hız = 0)        ║
║  Q / Ctrl+C → Çıkış                     ║
╚══════════════════════════════════════════╝
Sunucu: http://<robot-ip>:5000
"""

def _get_key(timeout=0.05):
    """
    Non-blocking tek karakter okuyucu.
    SSH raw-mode ile çalışır; timeout süresinde tuş yoksa None döner.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch.lower()
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def wasd_control_loop():
    """
    Ana WASD kontrol döngüsü.
    Fizik modeli gamepad sürümüyle aynı; sadece giriş kaynağı değişti.
    """
    global current_velocity, target_velocity, gear_direction, steering_angle, obstacle_detected

    ACCELERATION_RATE  = 3.0  
    COASTING_RATE      = 1.5 
    BRAKING_RATE       = 6.0  
    MAX_SPEED          = 100.0
    STEERING_STEP      = 0.15 
    STEERING_CENTER    = 0.10 

    key_w = False
    key_s = False
    key_a = False
    key_d = False

    print(HELP_TEXT)

    try:
        while True:
            obstacle_detected = (
                GPIO.input(MZ80_FRONT_PIN)       == 0 or
                GPIO.input(MZ80_LEFT_FRONT_PIN)  == 0 or
                GPIO.input(MZ80_RIGHT_FRONT_PIN) == 0 or
                GPIO.input(MZ80_RIGHT_REAR_PIN)  == 0 or
                GPIO.input(MZ80_LEFT_REAR_PIN)   == 0
            )

            ch = _get_key(timeout=0.05)   


            key_w = key_s = key_a = key_d = False

            if ch == 'w':
                key_w = True
            elif ch == 's':
                key_s = True
            elif ch == 'a':
                key_a = True
            elif ch == 'd':
                key_d = True
            elif ch == 'r':
                gear_direction *= -1
                mode = "İLERİ" if gear_direction == 1 else "GERİ"
                print(f"\r[VİTES] {mode}                    ")
            elif ch == ' ':
                current_velocity = 0.0
                target_velocity  = 0.0
                print("\r[ACİL DUR]                       ")
            elif ch in ('q', '\x03'):   # q veya Ctrl+C
                print("\nÇıkış yapılıyor...")
                break

            # ── Hız fiziği ────────────────────────────────
            if gear_direction == 1 and obstacle_detected:
                target_velocity  = 0.0
                current_velocity = 0.0
            else:
                if key_w:
                    target_velocity = MAX_SPEED
                else:
                    target_velocity = 0.0

                if key_s:
                    current_velocity = max(0.0, current_velocity - BRAKING_RATE)
                elif current_velocity < target_velocity:
                    current_velocity = min(target_velocity, current_velocity + ACCELERATION_RATE)
                elif current_velocity > target_velocity:
                    current_velocity = max(target_velocity, current_velocity - COASTING_RATE)

            current_velocity = max(0.0, min(MAX_SPEED, current_velocity))

            if key_a:
                steering_angle = max(-1.0, steering_angle - STEERING_STEP)
            elif key_d:
                steering_angle = min(1.0,  steering_angle + STEERING_STEP)
            else:
                if steering_angle > 0:
                    steering_angle = max(0.0, steering_angle - STEERING_CENTER)
                elif steering_angle < 0:
                    steering_angle = min(0.0, steering_angle + STEERING_CENTER)

            apply_motor_power()

            direction_arrow = "→ İLERİ" if gear_direction == 1 else "← GERİ"
            steering_bar    = f"{'◄' if steering_angle < -0.05 else ' '}{steering_angle:+.2f}{'►' if steering_angle > 0.05 else ' '}"
            obs_flag        = " ⚠ ENGEL!" if obstacle_detected else ""
            print(
                f"\r[{direction_arrow}] Hız:{current_velocity:5.1f}% "
                f"Direksiyon:{steering_bar}{obs_flag}        ",
                end="", flush=True
            )

    except KeyboardInterrupt:
        print("\nCtrl+C ile çıkış.")


try:
    threading.Thread(target=start_flask, daemon=True).start()
    wasd_control_loop()

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    try:
        camera.stop()
    except Exception:
        pass
    try:
        if ser:
            ser.close()
    except Exception:
        pass
    try:
        if i2c_bus:
            i2c_bus.close()
    except Exception:
        pass
    print("Tüm sistemler kapatıldı.")

