"""
cosmo.py - Cosmostreamer client for sitgs

Speaks the native Cosmostreamer UDP protocol (the one CosmoViewerNG uses). Checked against the
box firmware 0.24.11: examples/ClientExample/client-example.c, mqtt/telemetry-client.js and the
web UI (var/www/html/js/types.js, tpl/cams/mini2.js):

  * client broadcasts  [PACKET_DISCOVERY]  to port 5252 (and to any manual hosts)
  * each box answers   [PACKET_DISCOVERY_ANSWER, app_type, license, clients, name[40], ver[3]]
  * client sends       [PACKET_CLIENT_ALIVE, 1]  every second to each box it wants data from
  * box pushes         [PACKET_CAMERA_BINARY_TELEMETRY, ttype, ...]   (attitude / speeds / gimbal)
                       [PACKET_CAMERA_TEXT_TELEMETRY, "name=value,name=value,..."]
  * every datagram is wrapped:  <B start=0x52><B type=0><H payload_len><I counter><I client_id>  + payload

Telemetry values are dropped the way the firmware drops them: when camera_detected stops being 1,
or 2 s after the last binary telemetry. Also an optional minimal MQTT 3.1.1 subscriber for boxes
that publish through a broker (cosmo/<board_id>/telemetry and /status), e.g. over a VPN.

probe() is a standalone bring-up tool (sitgs --cosmo-probe): it finds boxes, decodes everything
they send and checks their web, waypoint-upload and RTSP ports.
"""
import collections, json, os, re, socket, struct, subprocess, sys, threading, time

START_BYTE = 0x52
(P_DISCOVERY, P_DISCOVERY_ANSWER, P_CLIENT_ALIVE, P_CLIENT_GONE, P_SERVER_STATUS, P_CLIENT_BINARY, P_BIN_TELEM, P_TEXT_TELEM,
 P_RC_MENU, P_PREVIEW_REQUEST, P_PREVIEW_ANSWER, P_WIRAL_STATUS, P_BT_GIMBAL_STATUS) = range(13)
PACKET_NAMES = {0: "discovery", 1: "discovery answer", 2: "client alive", 3: "client gone", 4: "server status", 5: "client binary",
                6: "binary telemetry", 7: "text telemetry", 8: "RC menu", 9: "preview request", 10: "preview answer",
                11: "Wiral status", 12: "BT gimbal status", 123: "JSON"}
DEFAULT_PORT = 5252
TELEMETRY_TIMEOUT_S = 2.0      # firmware: telemetry values cleared 2 s after the last binary telemetry

# app types from the firmware web UI (js/types.js)
APP_TYPES = {0: "DJI Osmo Wi-Fi", 1: "Osmo Pro/RAW wired", 2: "DJI Zenmuse", 3: "Pocket Osmo USB", 4: "Pocket Osmo Wi-Fi",
             5: "Osmo Action", 6: "GoPro", 8: "DJI Go4 drone", 13: "RTSP camera", 14: "Pocket 2 USB", 15: "Pocket 2 Wi-Fi",
             16: "Remote client", 17: "DJI FPV Goggles V1/V2", 18: "NDI receiver", 19: "RTMP receiver", 20: "Raven Eye",
             21: "Webcam", 22: "DJI Fly drone (RC-N)", 23: "DJI Goggles 2/3/Integra/N3", 25: "Osmo Action 2/3/4",
             26: "Pocket 3", 27: "AirPlay", 28: "DJI Pilot drone", 29: "DJI SkyPort", 30: "Goggles 3 wireless",
             31: "DJI SDR", 32: "DJI Fly drone (RC-N) Pro", 33: "DJI Goggles 2/3/Integra/N3 Pro", 34: "DJI Webcam",
             35: "Zhiyun Rider M", 36: "DJI Ronin", 37: "YouTube receiver", 38: "DJI RC Pro/Pro2"}
TELEMETRY_TYPES = {8, 22, 23, 28, 30, 32, 33}   # app types that carry drone GPS telemetry

def i16(b, i): return struct.unpack_from("<h", b, i)[0]

def frame(payload, counter=0):
    return struct.pack("<BBHII", START_BYTE, 0, len(payload), counter & 0xffffffff, 1) + payload

def local_broadcasts():
    """Directed broadcast address of every IPv4 interface. A 255.255.255.255 broadcast leaves macOS
    only through the primary interface, so a box on the other one (e.g. the box's own Wi-Fi while
    Ethernet is the default route) would never hear discovery."""
    out = set()
    try:
        if os.name == "nt":
            txt = subprocess.run(["ipconfig"], capture_output=True, text=True, timeout=5,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout
            ip = None
            for line in txt.splitlines():
                m = re.search(r"IPv4[^:]*:\s*([\d.]+)", line)
                if m: ip = m.group(1); continue
                m = re.search(r"Subnet Mask[^:]*:\s*([\d.]+)", line)
                if m and ip:
                    a = struct.unpack("!I", socket.inet_aton(ip))[0]; n = struct.unpack("!I", socket.inet_aton(m.group(1)))[0]
                    out.add(socket.inet_ntoa(struct.pack("!I", (a & n) | (~n & 0xffffffff)))); ip = None
        else:
            txt = subprocess.run(["ifconfig"], capture_output=True, text=True, timeout=5).stdout
            out.update(re.findall(r"inet [\d.]+ .*?broadcast ([\d.]+)", txt))
            out.update(re.findall(r"inet [\d.]+\s+netmask [\w.]+\s+broadcast ([\d.]+)", txt))
            if not out:   # Linux without net-tools
                txt = subprocess.run(["ip", "-4", "addr"], capture_output=True, text=True, timeout=5).stdout
                out.update(re.findall(r"inet [\d./]+ brd ([\d.]+)", txt))
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return sorted(b for b in out if not b.startswith("127."))

class Box:
    def __init__(self, host, port):
        self.host = host; self.port = port; self.key = "%s:%d" % (host, port)
        self.name = ""; self.app_type = None; self.license = None; self.clients = None; self.version = ""
        self.last_seen = 0.0; self.last_telemetry = 0.0; self.last_bin = 0.0; self.online = False
        self.values = {}      # text telemetry name -> str
        self.bin = {}         # binary telemetry
        self.drone_connected = False
        self.rx_hist = collections.deque(maxlen=600)
        self.trail = collections.deque(maxlen=2000)
        self.raw_text = collections.deque(maxlen=20)
        self.counts = collections.Counter()   # packet type name -> count
        self.keys_seen = {}                    # every text key ever received -> last value (survives clears)
        self.found_via = ""
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
        if self.values.get("gps_fix") == "0": return False
        la, lo = self.lat(), self.lng()
        return la is not None and lo is not None and (abs(la) > 0.0001 or abs(lo) > 0.0001)
    def alt_rel(self): return self.f("alt")
    def yaw(self): return self.bin.get("drone_yaw")
    def hspeed(self): return self.bin.get("drone_hspeed")
    def clear_telemetry(self):
        keep = self.values.get("camera_detected")
        self.values = {} if keep is None else {"camera_detected": keep}
        self.bin = {}
    def to_dict(self):
        v = self.values
        return {"key": self.key, "host": self.host, "port": self.port, "name": self.name, "app_type": self.app_type,
                "app_type_str": APP_TYPES.get(self.app_type, str(self.app_type)), "license": self.license, "version": self.version,
                "clients": self.clients, "online": self.online, "connected": self.connected(), "transport": self.transport,
                "found_via": self.found_via,
                "age_s": round(time.time() - self.last_telemetry, 1) if self.last_telemetry else None,
                "seen_s": round(time.time() - self.last_seen, 1) if self.last_seen else None,
                "lat": self.lat(), "lng": self.lng(), "fix": self.has_fix(), "alt_rel": self.alt_rel(),
                "sats": v.get("gps_sat_count"), "gps_fix": v.get("gps_fix"), "battery": v.get("battery_level"),
                "rc_battery": v.get("rc_battery_level"), "voltage": v.get("voltage"), "flight_mode": v.get("flight_mode"),
                "in_flight": v.get("in_flight"), "rth": v.get("rth"), "landing": v.get("landing"), "ftime_left": v.get("ftime_left"),
                "model": v.get("camera_model"), "sn": v.get("drone_sn"), "home_lat": self.f("gps_home_lat"), "home_lng": self.f("gps_home_lng"),
                "home_distance": v.get("home_distance"), "yaw": self.yaw(), "hspeed": self.hspeed(), "vspeed": self.bin.get("drone_vspeed"),
                "pan": self.bin.get("pan"), "tilt": self.bin.get("tilt"), "rec": self.bin.get("rec_state"), "takeoff_msl": self.takeoff_msl,
                "ctrl_app_en": v.get("ctrl_app_en"), "camera_detected": v.get("camera_detected"),
                "video_url": "rtsp://%s:554/video" % self.host}
    def diag(self):
        return {"key": self.key, "counts": dict(self.counts), "raw_text": list(self.raw_text)[-5:],
                "keys": dict(sorted(self.keys_seen.items())), "values_now": dict(sorted(self.values.items())),
                "binary": dict(sorted(self.bin.items()))}

class Capture:
    """Every datagram in and out, one JSON line each: {"t", "dir": "rx"|"tx", "peer": "host:port", "hex"}.
    Captures from a real box become test data and can be replayed with cosmo_sim.py --replay."""
    def __init__(self, path):
        self.path = path; self.lock = threading.Lock(); self.n = 0
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.fh = open(path, "a", buffering=1)
    def write(self, direction, peer, data):
        line = json.dumps({"t": round(time.time(), 3), "dir": direction, "peer": "%s:%d" % peer, "hex": data.hex()})
        with self.lock:
            if self.fh: self.fh.write(line + "\n"); self.n += 1
    def close(self):
        with self.lock:
            if self.fh: self.fh.close(); self.fh = None

class CosmoClient:
    """Discovers boxes, keeps them alive, parses their telemetry. Thread-safe enough for a 1 Hz UI."""
    def __init__(self, get_cfg, log, capture_path=None):
        self.get_cfg = get_cfg; self.log = log; self.capture_path = capture_path
        self.boxes = {}; self.lock = threading.Lock()
        self.sock = None; self.counter = 0; self.running = False; self.errors = 0; self.packets = 0
        self.mqtt = None; self.capture = None
        self.drops = collections.Counter()          # why datagrams were not used
        self.unknown = collections.deque(maxlen=20) # (time, peer, hex) of packets we could not use
        self.bcast = []; self.bcast_at = 0.0
        self.last_capture = None

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
        self.set_capture(False)

    def set_capture(self, on):
        if on and self.capture is None and self.capture_path:
            path = self.capture_path() if callable(self.capture_path) else self.capture_path
            self.capture = Capture(path); self.log("cosmo capture started %s" % path)
        elif not on and self.capture is not None:
            c = self.capture; self.capture = None; c.close(); self.last_capture = c.path; self.log("cosmo capture stopped (%d datagrams) %s" % (c.n, c.path))

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", 0)); s.settimeout(0.5)
        return s

    def _send(self, payload, host, port):
        if not self.sock: return
        data = frame(payload, self.counter); self.counter += 1
        try:
            self.sock.sendto(data, (host, port))
            if self.capture: self.capture.write("tx", (host, port), data)
        except OSError as e:
            self.errors += 1
            if self.errors <= 5: self.log("cosmo send error to %s:%s: %s" % (host, port, e))

    def manual_targets(self, cfg):
        targets = []
        for h in cfg.get("manual_hosts", []):
            h = h.strip()
            if not h: continue
            host, _, port = h.partition(":")
            try: targets.append((host, int(port) if port else DEFAULT_PORT))
            except ValueError: continue
        return targets

    # --- threads
    def tx_loop(self):
        while self.running:
            try: self.tx_once()
            except Exception as e: self.log("cosmo tx error: %s" % e)
            time.sleep(1)

    def tx_once_capture(self):
        cfg = self.get_cfg(); self.set_capture(bool(cfg.get("capture")) and bool(cfg.get("enabled")))

    def tx_once(self):
        self.tx_once_capture(); cfg = self.get_cfg()
        if not cfg.get("enabled"): return
        if self.sock is None:
            try: self.sock = self._open()
            except OSError as e: self.log("cosmo socket error: %s" % e); time.sleep(1); return
        targets = set(self.manual_targets(cfg))
        if cfg.get("discovery", True):
            if time.time() - self.bcast_at > 30: self.bcast = local_broadcasts(); self.bcast_at = time.time()
            targets.add(("255.255.255.255", DEFAULT_PORT))
            targets.update((b, DEFAULT_PORT) for b in self.bcast)
        for host, port in targets: self._send(bytes([P_DISCOVERY]), host, port)
        alive = []; now = time.time()
        with self.lock:
            for b in list(self.boxes.values()):
                if b.transport != "udp": continue
                if now - b.last_seen > 4 and b.online:
                    b.online = False; self.log("cosmo box offline: %s (%s)" % (b.name, b.key))
                if b.last_bin and now - b.last_bin > TELEMETRY_TIMEOUT_S:
                    b.last_bin = 0.0; b.drone_connected = False; b.clear_telemetry()
                    self.log("cosmo %s: telemetry timeout, values cleared" % (b.name or b.key))
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
            if self.capture: self.capture.write("rx", addr, data)
            if len(data) < 13 or data[0] != START_BYTE or data[1] != 0:
                self.drops["bad header"] += 1; self.unknown.append((time.time(), "%s:%d" % addr, data[:64].hex())); continue
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
            ptype = p[0]
            if ptype == P_DISCOVERY_ANSWER:
                if b is None:
                    b = self.boxes[key] = Box(host, port)
                    b.found_via = "manual host" if host in {h for h, _ in self.manual_targets(self.get_cfg())} else "broadcast"
                b.counts["discovery answer"] += 1
                b.app_type = p[1]; b.license = p[2]; b.clients = p[3]
                b.name = p[4:44].split(b"\0")[0].decode("ascii", "replace").strip() or b.name
                if len(p) >= 47: b.version = "%d.%d.%d" % (p[44], p[45], p[46])
                if not b.online: self.log("cosmo box online: %s %s (%s, licence byte %s) v%s via %s" % (
                    b.name, b.key, APP_TYPES.get(b.app_type, b.app_type), b.license, b.version, b.found_via))
                b.online = True; b.last_seen = now; return
            if b is None:
                self.drops["unknown host"] += 1; self.unknown.append((now, key, p[:64].hex())); return
            b.last_seen = now; b.online = True
            b.counts[PACKET_NAMES.get(ptype, "type %d" % ptype)] += 1
            if ptype == P_SERVER_STATUS:
                if len(p) > 3: b.app_type = p[1]; b.license = p[2]; b.clients = p[3]
            elif ptype == P_BIN_TELEM:
                b.last_bin = b.last_telemetry = now
                ttype = p[1] if len(p) > 1 else None
                if ttype != 0: b.counts["binary ttype %s" % ttype] += 1; self.unknown.append((now, key, p[:64].hex()))
                self.parse_binary(b, p)
            elif ptype == P_TEXT_TELEM:
                txt = p[1:].decode("ascii", "replace"); b.raw_text.append(txt)
                self.parse_text(b, txt); b.last_telemetry = now
                self.after_text(b, now)
            elif ptype not in (P_CLIENT_ALIVE, P_CLIENT_GONE):
                self.unknown.append((now, key, p[:64].hex()))

    @staticmethod
    def parse_binary(b, d):
        """Binary telemetry layout of the DJI Fly app type (firmware parse_telemetry_22). Fields beyond
        the end of a shorter packet are skipped rather than failing the whole packet."""
        v = b.bin; n = len(d)
        def put(name, off, scale=10.0):
            if off + 2 <= n: v[name] = i16(d, off) / scale
        put("pan", 3); put("tilt", 5); put("roll", 7)
        if n > 9: v["photo_state"] = (d[9] >> 3) & 1; v["rec_state"] = 1 if ((d[9] >> 6) & 3) else 0
        put("drone_yaw", 37); put("drone_pitch", 39); put("drone_roll", 41)
        put("drone_speed1", 43); put("drone_speed2", 45); put("drone_speed3", 47)
        put("drone_hspeed", 51); put("drone_vspeed", 53)

    @staticmethod
    def parse_text(b, txt):
        for part in txt.split(","):
            if "=" not in part: continue
            k, _, val = part.partition("="); k = k.strip()
            if k: b.values[k] = val.strip(); b.keys_seen[k] = val.strip()
        # firmware: values are dropped when the drone disconnects (camera_detected stops being 1)
        if b.values.get("camera_detected") == "1": b.drone_connected = True
        elif b.drone_connected:
            b.drone_connected = False; b.clear_telemetry()

    def after_text(self, b, now):
        if b.has_fix():
            b.rx_hist.append((now, None, None)); b.trail.append((now, b.lat(), b.lng(), b.alt_rel()))

    # --- MQTT path
    def on_mqtt(self, topic, payload):
        parts = topic.split("/")
        if len(parts) != 3 or parts[0] != "cosmo": return
        board = parts[1]; key = "mqtt:" + board[:12]; now = time.time()
        with self.lock:
            b = self.boxes.get(key)
            if b is None:
                b = self.boxes[key] = Box("mqtt", 0); b.key = key; b.transport = "mqtt"; b.name = board[:12]; b.found_via = "MQTT"
            b.last_seen = now; b.online = True; b.counts["mqtt " + parts[2]] += 1
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
                    else: b.values[k] = str(val); b.keys_seen[k] = str(val)
                if j.get("callsign"): b.name = j["callsign"]
                b.last_telemetry = now; self.after_text(b, now)

    # --- commands
    def send_text_command(self, key, cmd):
        with self.lock: b = self.boxes.get(key)
        if b and b.transport == "udp": self._send(cmd.encode(), b.host, b.port)

    def snapshot(self):
        with self.lock:
            return {"boxes": [b.to_dict() for b in sorted(self.boxes.values(), key=lambda x: (x.name, x.key))], "packets": self.packets,
                    "errors": self.errors, "dropped": sum(self.drops.values()), "capturing": self.capture is not None,
                    "mqtt": self.mqtt.state if self.mqtt else None}

    def diag(self):
        with self.lock:
            return {"packets": self.packets, "errors": self.errors, "drops": dict(self.drops),
                    "unknown": [{"t": round(t, 1), "peer": peer, "hex": h} for t, peer, h in self.unknown],
                    "broadcasts": ["255.255.255.255"] + self.bcast,
                    "capture": {"on": self.capture is not None, "path": self.capture.path if self.capture else None,
                                "datagrams": self.capture.n if self.capture else 0},
                    "boxes": [b.diag() for b in sorted(self.boxes.values(), key=lambda x: (x.name, x.key))]}

    def box(self, key):
        with self.lock: return self.boxes.get(key)


# ------------------------------------------------------------------------- bring-up probe
def _http(url, timeout=4):
    import urllib.request, urllib.error
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read(400).decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.code, (e.read(200) or b"").decode("utf-8", "replace")
    except (OSError, ValueError) as e:
        return None, str(e)

def _rtsp(host, port=554, timeout=4):
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall(("OPTIONS rtsp://%s:%d/video RTSP/1.0\r\nCSeq: 1\r\nUser-Agent: sitgs-probe\r\n\r\n" % (host, port)).encode())
            s.settimeout(timeout); return (s.recv(300).decode("ascii", "replace").splitlines() or ["(no reply)"])[0]
    except OSError as e:
        return "not reachable: %s" % e

def probe(hosts=(), seconds=20, capture=None, http=True, out=print):
    """Bring-up check for a real box: discovery (broadcast, every interface's broadcast, given hosts),
    keep-alive for `seconds`, decode and summarise everything received, then probe HTTP / upload / RTSP.
    Returns the report dict; also prints it as it goes."""
    targets = [(h.partition(":")[0], int(h.partition(":")[2] or DEFAULT_PORT)) for h in hosts if h.strip()]
    bcasts = local_broadcasts()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1); s.bind(("0.0.0.0", 0)); s.settimeout(0.2)
    cap = Capture(capture) if capture else None
    out("sitgs Cosmostreamer probe - local port %d" % s.getsockname()[1])
    out("discovery to 255.255.255.255%s%s" % ("".join(", " + b for b in bcasts), "".join(", %s:%d" % t for t in targets)))
    boxes = {}; others = collections.Counter(); counter = 0; t0 = time.time(); last_tx = 0.0
    def send(payload, addr):
        nonlocal counter
        data = frame(payload, counter); counter += 1
        try:
            s.sendto(data, addr)
            if cap: cap.write("tx", addr, data)
        except OSError as e: out("  send to %s:%d failed: %s" % (addr[0], addr[1], e))
    while time.time() - t0 < seconds:
        if time.time() - last_tx >= 1:
            last_tx = time.time()
            for a in [("255.255.255.255", DEFAULT_PORT)] + [(b, DEFAULT_PORT) for b in bcasts] + targets: send(bytes([P_DISCOVERY]), a)
            for b in boxes.values(): send(bytes([P_CLIENT_ALIVE, 1]), (b["host"], b["port"]))
        try: data, addr = s.recvfrom(65535)
        except socket.timeout: continue
        except OSError: continue
        if cap: cap.write("rx", addr, data)
        if len(data) < 13 or data[0] != START_BYTE or data[1] != 0:
            others["not a Cosmostreamer packet from %s" % addr[0]] += 1; continue
        p = data[12:]; key = "%s:%d" % addr; b = boxes.get(key)
        if b is None:
            b = boxes[key] = {"host": addr[0], "port": addr[1], "answered": False, "types": collections.Counter(), "keys": {},
                              "binary": collections.Counter(), "unknown": [], "raw_text": [], "first_s": round(time.time() - t0, 1)}
        b["types"][PACKET_NAMES.get(p[0], "type %d" % p[0])] += 1
        if p[0] == P_DISCOVERY_ANSWER:
            first = not b["answered"]
            b.update(answered=True, app_type=p[1] if len(p) > 1 else None, license=p[2] if len(p) > 2 else None,
                     clients=p[3] if len(p) > 3 else None, name=p[4:44].split(b"\0")[0].decode("ascii", "replace").strip(),
                     version="%d.%d.%d" % (p[44], p[45], p[46]) if len(p) >= 47 else "", answer_len=len(p))
            if first: out("  answer from %s: %r  %s (app type %s), licence byte %s, firmware %s, clients %s" % (
                key, b["name"], APP_TYPES.get(b["app_type"], "?"), b["app_type"], b["license"], b["version"], b["clients"]))
        elif p[0] == P_TEXT_TELEM:
            txt = p[1:].decode("ascii", "replace")
            if len(b["raw_text"]) < 5: b["raw_text"].append(txt)
            for part in txt.split(","):
                k, eq, v = part.partition("=")
                if eq and k.strip(): b["keys"][k.strip()] = v.strip()
        elif p[0] == P_BIN_TELEM:
            b["binary"]["ttype %s, %d bytes" % (p[1] if len(p) > 1 else "?", len(p))] += 1
        elif p[0] not in (P_SERVER_STATUS,) and len(b["unknown"]) < 5:
            b["unknown"].append(p[:64].hex())
    if cap: cap.close()
    report = {"seconds": seconds, "broadcasts": bcasts, "manual_hosts": ["%s:%d" % t for t in targets], "boxes": [], "other": dict(others)}
    out("\n=== %d box(es) heard in %d s" % (sum(1 for b in boxes.values() if b["answered"]), seconds))
    for key, b in boxes.items():
        r = {k: (dict(v) if isinstance(v, collections.Counter) else v) for k, v in b.items()}
        out("\n%s  %s" % (key, "%r %s (app type %s) licence byte %s firmware %s" % (b.get("name"), APP_TYPES.get(b.get("app_type"), "?"),
            b.get("app_type"), b.get("license"), b.get("version")) if b["answered"] else "(never sent a discovery answer)"))
        if b["port"] != DEFAULT_PORT: out("  NOTE replies come from port %d, not %d" % (b["port"], DEFAULT_PORT))
        out("  packets: " + ", ".join("%s %d" % kv for kv in sorted(b["types"].items())))
        if b["binary"]: out("  binary telemetry: " + ", ".join("%s x%d" % kv for kv in sorted(b["binary"].items())))
        if b["keys"]:
            out("  text keys (%d), last values:" % len(b["keys"]))
            for k in sorted(b["keys"]): out("    %-22s %s" % (k, b["keys"][k]))
        else: out("  no text telemetry (normal without a drone connected)")
        for h in b["unknown"]: out("  unrecognised payload: %s" % h)
        if http and b["answered"]:
            st, body = _http("http://%s/" % b["host"])
            title = re.search(r"<title>(.*?)</title>", body or "", re.I | re.S)
            r["http"] = {"status": st, "title": title.group(1).strip() if title else None}
            out("  web UI    http://%s/ -> %s %s" % (b["host"], st or "no answer", (r["http"]["title"] or body[:60] if st else body)))
            st, body = _http("http://%s/waypoints.php?action=get&id=sitgs_probe" % b["host"])
            r["waypoints"] = {"status": st, "body": (body or "")[:80]}
            out("  waypoints http://%s/waypoints.php -> %s %s%s" % (b["host"], st or "no answer", (body or "")[:60].strip(),
                "  (HTTP login required)" if st == 401 else ""))
            r["rtsp"] = _rtsp(b["host"])
            out("  RTSP      rtsp://%s:554/video -> %s%s" % (b["host"], r["rtsp"],
                "  (switch RTSP on in the box web UI)" if "not reachable" in r["rtsp"] else ""))
        report["boxes"].append(r)
    for what, n in others.items(): out("ignored: %s x%d" % (what, n))
    if not boxes:
        out("\nNo box answered. Check: the computer is on the box's network (box Wi-Fi: box at 192.168.50.1),"
            " the firewall allows incoming UDP for this program, and try --probe-host 192.168.50.1")
    if capture: out("\ncapture written to %s" % capture)
    return report


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
