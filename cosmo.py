"""
cosmo.py - Cosmostreamer client for sitgs

Speaks the native Cosmostreamer UDP protocol (the one CosmoViewerNG uses; documented in the
firmware's examples/ClientExample/client-example.c):

  * client broadcasts  [PACKET_DISCOVERY]  to 255.255.255.255:5252 (and to any manual hosts)
  * each box answers   [PACKET_DISCOVERY_ANSWER, app_type, license, clients, name[40], ver[3]]
  * client sends       [PACKET_CLIENT_ALIVE, 1]  every second to each box it wants data from
  * box pushes         [PACKET_CAMERA_BINARY_TELEMETRY, ttype, ...]   (attitude / speeds / gimbal)
                       [PACKET_CAMERA_TEXT_TELEMETRY, "name=value,name=value,..."]
  * every datagram is wrapped:  <B start=0x52><B type=0><H payload_len><I counter><I client_id>  + payload

Also an optional minimal MQTT 3.1.1 subscriber for boxes that publish through a broker
(cosmo/<board_id>/telemetry and /status), e.g. over a VPN.
"""
import collections, json, socket, struct, threading, time

START_BYTE = 0x52
P_DISCOVERY, P_DISCOVERY_ANSWER, P_CLIENT_ALIVE, P_CLIENT_GONE, P_SERVER_STATUS, P_CLIENT_BINARY, P_BIN_TELEM, P_TEXT_TELEM = range(8)
DEFAULT_PORT = 5252

APP_TYPES = {0: "DJI Osmo Wi-Fi", 1: "Osmo Pro/RAW wired", 2: "DJI Zenmuse", 3: "Pocket Osmo USB", 4: "Pocket Osmo Wi-Fi",
             5: "Osmo Action", 6: "GoPro", 8: "DJI Go4 drone", 13: "RTSP camera", 14: "Pocket 2 USB", 15: "Pocket 2 Wi-Fi",
             16: "Remote client", 17: "DJI FPV Goggles V1/V2", 18: "NDI receiver", 19: "RTMP receiver", 20: "Raven Eye",
             21: "Webcam", 22: "DJI Fly drone (RC-N)", 23: "DJI Goggles 2/3/Integra/N3", 25: "Osmo Action 2/3/4",
             26: "Pocket 3", 27: "AirPlay", 28: "DJI Pilot drone", 29: "DJI SkyPort", 30: "Goggles 3 wireless",
             31: "DJI SDR", 38: "DJI RC Pro/Pro2"}
TELEMETRY_TYPES = {22, 23, 30, 8, 28}   # app types that carry drone GPS telemetry

def i16(b, i): return struct.unpack_from("<h", b, i)[0]

class Box:
    def __init__(self, host, port):
        self.host = host; self.port = port; self.key = "%s:%d" % (host, port)
        self.name = ""; self.app_type = None; self.license = None; self.clients = None; self.version = ""
        self.last_seen = 0.0; self.last_telemetry = 0.0; self.online = False
        self.values = {}      # text telemetry name -> str
        self.bin = {}         # binary telemetry
        self.rx_hist = collections.deque(maxlen=600)
        self.trail = collections.deque(maxlen=2000)
        self.raw_text = collections.deque(maxlen=20)
        self.transport = "udp"
        self.takeoff_msl = None

    # --- derived
    def connected(self):
        return self.online and self.values.get("camera_detected") == "1" and (time.time() - self.last_telemetry) < 2.5
    def f(self, k):
        try: return float(self.values[k])
        except (KeyError, ValueError, TypeError): return None
    def lat(self): return self.f("gps_lat")
    def lng(self): return self.f("gps_lng")
    def has_fix(self):
        la, lo = self.lat(), self.lng()
        return la is not None and lo is not None and (abs(la) > 0.0001 or abs(lo) > 0.0001)
    def alt_rel(self): return self.f("alt")
    def yaw(self): return self.bin.get("drone_yaw")
    def hspeed(self): return self.bin.get("drone_hspeed")
    def to_dict(self):
        v = self.values
        return {"key": self.key, "host": self.host, "port": self.port, "name": self.name, "app_type": self.app_type,
                "app_type_str": APP_TYPES.get(self.app_type, str(self.app_type)), "license": self.license, "version": self.version,
                "clients": self.clients, "online": self.online, "connected": self.connected(), "transport": self.transport,
                "age_s": round(time.time() - self.last_telemetry, 1) if self.last_telemetry else None,
                "seen_s": round(time.time() - self.last_seen, 1) if self.last_seen else None,
                "lat": self.lat(), "lng": self.lng(), "fix": self.has_fix(), "alt_rel": self.alt_rel(),
                "sats": v.get("gps_sat_count"), "gps_fix": v.get("gps_fix"), "battery": v.get("battery_level"),
                "rc_battery": v.get("rc_battery_level"), "voltage": v.get("voltage"), "flight_mode": v.get("flight_mode"),
                "in_flight": v.get("in_flight"), "rth": v.get("rth"), "landing": v.get("landing"), "ftime_left": v.get("ftime_left"),
                "model": v.get("camera_model"), "sn": v.get("drone_sn"), "home_lat": self.f("gps_home_lat"), "home_lng": self.f("gps_home_lng"),
                "home_distance": v.get("home_distance"), "yaw": self.yaw(), "hspeed": self.hspeed(), "vspeed": self.bin.get("drone_vspeed"),
                "pan": self.bin.get("pan"), "tilt": self.bin.get("tilt"), "rec": self.bin.get("rec_state"), "takeoff_msl": self.takeoff_msl,
                "video_url": "rtsp://%s:554/video" % self.host}

class CosmoClient:
    """Discovers boxes, keeps them alive, parses their telemetry. Thread-safe enough for a 1 Hz UI."""
    def __init__(self, get_cfg, log):
        self.get_cfg = get_cfg; self.log = log
        self.boxes = {}; self.lock = threading.Lock()
        self.sock = None; self.counter = 0; self.running = False; self.errors = 0; self.packets = 0
        self.mqtt = None

    # --- lifecycle
    def start(self):
        if self.running: return
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True); self.rx_thread.start()
        self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True); self.tx_thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except OSError: pass
        self.sock = None
        if self.mqtt: self.mqtt.stop(); self.mqtt = None

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", 0)); s.settimeout(0.5)
        return s

    def _send(self, payload, host, port):
        if not self.sock: return
        hdr = struct.pack("<BBHII", START_BYTE, 0, len(payload), self.counter & 0xffffffff, 1); self.counter += 1
        try: self.sock.sendto(hdr + payload, (host, port))
        except OSError as e:
            self.errors += 1
            if self.errors <= 5: self.log("cosmo send error to %s:%s: %s" % (host, port, e))

    # --- threads
    def tx_loop(self):
        while self.running:
            try: self.tx_once()
            except Exception as e: self.log("cosmo tx error: %s" % e)
            time.sleep(1)

    def tx_once(self):
        cfg = self.get_cfg()
        if not cfg.get("enabled"): return
        if self.sock is None:
            try: self.sock = self._open()
            except OSError as e: self.log("cosmo socket error: %s" % e); time.sleep(1); return
        targets = set()
        if cfg.get("discovery", True): targets.add(("255.255.255.255", DEFAULT_PORT))
        for h in cfg.get("manual_hosts", []):
            h = h.strip()
            if not h: continue
            host, _, port = h.partition(":")
            try: targets.add((host, int(port) if port else DEFAULT_PORT))
            except ValueError: continue
        for host, port in targets: self._send(bytes([P_DISCOVERY]), host, port)
        alive = []
        with self.lock:
            for b in list(self.boxes.values()):
                if b.transport != "udp": continue
                if time.time() - b.last_seen > 4 and b.online:
                    b.online = False; self.log("cosmo box offline: %s (%s)" % (b.name, b.key))
                if b.online: alive.append((b.host, b.port))
        # sent outside the lock: a slow sendto must not stall snapshot() and, through it, the serial thread
        for host, port in alive: self._send(bytes([P_CLIENT_ALIVE, 1]), host, port)
        m = cfg.get("mqtt", {})
        if m.get("enabled") and self.mqtt is None:
            self.mqtt = MqttSubscriber(m, self.on_mqtt, self.log); self.mqtt.start()
        elif (not m.get("enabled")) and self.mqtt is not None:
            self.mqtt.stop(); self.mqtt = None

    def rx_loop(self):
        while self.running:
            s = self.sock
            if s is None: time.sleep(0.2); continue
            try: data, addr = s.recvfrom(65535)
            except socket.timeout: continue
            except OSError: time.sleep(0.2); continue
            if len(data) < 13 or data[0] != START_BYTE or data[1] != 0: continue
            self.packets += 1
            try: self.on_packet(data[12:], addr[0], addr[1])
            except Exception as e:
                self.errors += 1
                if self.errors <= 20: self.log("cosmo bad packet from %s:%s (%d bytes): %s" % (addr[0], addr[1], len(data), e))

    # --- parsing
    def on_packet(self, p, host, port):
        key = "%s:%d" % (host, port); now = time.time()
        with self.lock:
            b = self.boxes.get(key)
            if p[0] == P_DISCOVERY_ANSWER:
                if b is None:
                    b = self.boxes[key] = Box(host, port)
                b.app_type = p[1]; b.license = p[2]; b.clients = p[3]
                b.name = p[4:44].split(b"\0")[0].decode("ascii", "replace").strip() or b.name
                if len(p) >= 47: b.version = "%d.%d.%d" % (p[44], p[45], p[46])
                if not b.online: self.log("cosmo box online: %s %s (%s) v%s" % (b.name, b.key, APP_TYPES.get(b.app_type, b.app_type), b.version))
                b.online = True; b.last_seen = now; return
            if b is None: return
            b.last_seen = now; b.online = True
            if p[0] == P_SERVER_STATUS:
                if len(p) > 3: b.app_type = p[1]; b.license = p[2]; b.clients = p[3]
            elif p[0] == P_BIN_TELEM and len(p) >= 55 and p[1] == 0:
                self.parse_binary(b, p); b.last_telemetry = now
            elif p[0] == P_TEXT_TELEM:
                txt = p[1:].decode("ascii", "replace"); b.raw_text.append(txt)
                self.parse_text(b, txt); b.last_telemetry = now
                self.after_text(b, now)

    def parse_binary(self, b, d):
        v = b.bin
        v["pan"] = i16(d, 3) / 10.0; v["tilt"] = i16(d, 5) / 10.0; v["roll"] = i16(d, 7) / 10.0
        v["photo_state"] = (d[9] >> 3) & 1; v["rec_state"] = 1 if ((d[9] >> 6) & 3) else 0
        v["drone_yaw"] = i16(d, 37) / 10.0; v["drone_pitch"] = i16(d, 39) / 10.0; v["drone_roll"] = i16(d, 41) / 10.0
        v["drone_speed1"] = i16(d, 43) / 10.0; v["drone_speed2"] = i16(d, 45) / 10.0; v["drone_speed3"] = i16(d, 47) / 10.0
        v["drone_hspeed"] = i16(d, 51) / 10.0; v["drone_vspeed"] = i16(d, 53) / 10.0

    def parse_text(self, b, txt):
        for part in txt.split(","):
            if "=" not in part: continue
            k, _, val = part.partition("="); k = k.strip()
            if k: b.values[k] = val.strip()

    def after_text(self, b, now):
        if b.has_fix():
            b.rx_hist.append((now, None, None)); b.trail.append((now, b.lat(), b.lng(), b.alt_rel()))
        # remember take-off MSL: when the drone reports not in flight its GPS-alt is unknown, so we keep the tracker's MSL (set by fusion)

    # --- MQTT path
    def on_mqtt(self, topic, payload):
        parts = topic.split("/")
        if len(parts) != 3 or parts[0] != "cosmo": return
        board = parts[1]; key = "mqtt:" + board[:12]; now = time.time()
        with self.lock:
            b = self.boxes.get(key)
            if b is None:
                b = self.boxes[key] = Box("mqtt", 0); b.key = key; b.transport = "mqtt"; b.name = board[:12]
            b.last_seen = now; b.online = True
            try: j = json.loads(payload)
            except ValueError: return
            if parts[2] == "status":
                if j.get("callsign"): b.name = j["callsign"]
                if j.get("model"): b.values["camera_model"] = j["model"]
                if j.get("sn"): b.values["drone_sn"] = j["sn"]
                b.values["camera_detected"] = "1" if j.get("connected") else "0"
            elif parts[2] == "telemetry":
                for k, val in j.items():
                    if isinstance(val, (int, float)) and k in ("pan", "tilt", "roll", "drone_yaw", "drone_pitch", "drone_roll", "drone_hspeed", "drone_vspeed", "rec_state", "photo_state"):
                        b.bin[k] = float(val)
                    else: b.values[k] = str(val)
                if j.get("callsign"): b.name = j["callsign"]
                b.last_telemetry = now; self.after_text(b, now)

    # --- commands (not exposed in the UI yet; here for completeness)
    def send_text_command(self, key, cmd):
        with self.lock: b = self.boxes.get(key)
        if b and b.transport == "udp": self._send(cmd.encode(), b.host, b.port)

    def snapshot(self):
        with self.lock:
            return {"boxes": [b.to_dict() for b in sorted(self.boxes.values(), key=lambda x: (x.name, x.key))], "packets": self.packets,
                    "errors": self.errors, "mqtt": self.mqtt.state if self.mqtt else None}

    def box(self, key):
        with self.lock: return self.boxes.get(key)


class MqttSubscriber:
    """Tiny MQTT 3.1.1 client: CONNECT, SUBSCRIBE cosmo/#, PINGREQ, PUBLISH parsing. No TLS, QoS 0."""
    def __init__(self, cfg, on_message, log):
        self.cfg = cfg; self.on_message = on_message; self.log = log; self.running = False; self.sock = None; self.state = "starting"
    def start(self):
        self.running = True; threading.Thread(target=self.loop, daemon=True).start()
    def stop(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except OSError: pass
    @staticmethod
    def _str(s): b = s.encode(); return struct.pack("!H", len(b)) + b
    @staticmethod
    def _enc_len(n):
        out = b""
        while True:
            d = n % 128; n //= 128
            out += bytes([d | (0x80 if n else 0)])
            if not n: return out
    def loop(self):
        while self.running:
            try:
                host = self.cfg.get("host", "127.0.0.1"); port = int(self.cfg.get("port", 1883) or 1883)
                self.state = "connecting %s:%d" % (host, port)
                s = socket.create_connection((host, port), timeout=5); self.sock = s; s.settimeout(1)
                flags = 0x02; body = self._str("MQTT") + bytes([4, 0]) + struct.pack("!H", 30)
                user, pw = self.cfg.get("username", ""), self.cfg.get("password", "")
                if user: flags |= 0x80
                if pw: flags |= 0x40
                body = self._str("MQTT") + bytes([4, flags]) + struct.pack("!H", 30) + self._str("sitgs-%d" % (time.time() % 100000))
                if user: body += self._str(user)
                if pw: body += self._str(pw)
                s.sendall(bytes([0x10]) + self._enc_len(len(body)) + body)
                sub = struct.pack("!H", 1) + self._str("cosmo/#") + bytes([0])
                s.sendall(bytes([0x82]) + self._enc_len(len(sub)) + sub)
                self.state = "connected %s:%d" % (host, port); self.log("mqtt connected %s:%d" % (host, port))
                buf = b""; last_ping = time.time()
                while self.running:
                    try:
                        chunk = s.recv(65535)
                        if chunk == b"": raise OSError("broker closed connection")
                        buf += chunk
                    except socket.timeout:
                        pass
                    if time.time() - last_ping > 15:
                        s.sendall(b"\xc0\x00"); last_ping = time.time()
                    while len(buf) >= 2:
                        # decode remaining length
                        mult = 1; rl = 0; i = 1
                        while True:
                            if i >= len(buf): rl = None; break
                            d = buf[i]; rl += (d & 127) * mult; mult *= 128; i += 1
                            if not (d & 128): break
                        if rl is None or len(buf) < i + rl: break
                        pkt = buf[i:i + rl]; ptype = buf[0] >> 4; buf = buf[i + rl:]
                        if ptype == 3:  # PUBLISH
                            tl = struct.unpack("!H", pkt[:2])[0]; topic = pkt[2:2 + tl].decode("utf8", "replace"); rest = pkt[2 + tl:]
                            try: self.on_message(topic, rest.decode("utf8", "replace"))
                            except Exception as e: self.log("mqtt handler error: %s" % e)
            except (OSError, struct.error) as e:
                self.state = "error: %s" % e; self.log("mqtt error: %s" % e)
            finally:
                try:
                    if self.sock: self.sock.close()
                except OSError: pass
                self.sock = None
            if self.running: time.sleep(3)
