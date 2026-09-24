#!/usr/bin/env python3
"""
sitgs - SIT/SS GPS-LoRa tracker ground station
Web reimplementation of the LabVIEW "LoRa GPS RX and Logger" client, tab for tab.

  python3 sitgs.py                      -> http://localhost:8090  (config: sitgs.json)
  python3 sitgs.py --list-ports
  python3 sitgs.py --replay capture.nmea [--speed 1]

Dependency: pyserial (a copy in ./serial works too)
"""
import argparse, collections, csv, datetime as dt, json, math, os, socket, struct, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from apppaths import RES_DIR, DATA_DIR
try:
    import serial, serial.tools.list_ports
except ImportError:
    serial = None
from cosmo import CosmoClient, TELEMETRY_TYPES
import mission as mission_mod

DEFAULT_CONFIG = {
    "connection": {"type": "serial", "port": "auto", "baud": 115200, "host": "192.168.1.50", "tcp_port": 4001,
                   "auto_connect": True, "record_raw": ""},
    "log": {"nmea_enabled": True, "nmea_dir": "logs/nmea", "split_enabled": True, "split_dir": "logs/csv",
            "event_name": "TEST", "inject_rx_timestamp": True},
    "barometer": {"mode": "disable", "ref_pressure_bar": 1.01325, "ref_temp_c": 15.0, "ref_altitude_m": 0.0,
                  "auto_update": "disable", "ref_tracker_id": 0},
    "cot": {"enabled": True, "type": "a-f-A-C-H-q", "uid_prefix": "TRK.", "uid_format": "%03d",
            "period_ms": 500, "stale_s": 30, "geoid_height_m": 34.0, "geoid_auto_update": True,
            "include_flow_tag": False,
            "destinations": [
                {"enabled": True, "host": "239.2.3.1", "port": 6969, "iface": "0.0.0.0", "ttl": 4},
                {"enabled": True, "host": "127.0.0.1", "port": 1800, "iface": "0.0.0.0", "ttl": 2}]},
    "kml": {"enabled": False, "path": "logs/kml/tracks.kml", "geojson_path": "logs/kml/tracks.geojson",
            "interval_s": 2, "trail_points": 300},
    "web": {"port": 8090},
    "data_age_tolerance_s": 30,
    "tdma_slot_ms": 50,
    "filter": {"receiver_address": 111, "require_payload": True},
    "cosmo": {"enabled": True, "discovery": True, "manual_hosts": [],
              "mqtt": {"enabled": False, "host": "127.0.0.1", "port": 1883, "username": "", "password": ""},
              "pairs": {}, "video_in_cot": True, "cot_unpaired": True, "unpaired_uid_prefix": "DJI.", "dji_stale_s": 2.5,
              "mission_cmds": {"start": "!{\"cmd\":\"drone\",\"param\":\"mission\",\"value\":\"start\",\"value2\":\"%NAME%.kmz\"}",
                               "pause": "!{\"cmd\":\"drone\",\"param\":\"mission\",\"value\":\"pause\"}",
                               "resume": "!{\"cmd\":\"drone\",\"param\":\"mission\",\"value\":\"resume\"}",
                               "stop": "!{\"cmd\":\"drone\",\"param\":\"mission\",\"value\":\"stop\"}",
                               "rth": "!{\"cmd\":\"drone\",\"param\":\"rth\",\"value\":\"\"}",
                               "land": "!{\"cmd\":\"drone\",\"param\":\"landing\",\"value\":\"\"}",
                               "take_control": "!{\"cmd\":\"drone\",\"param\":\"control_app\",\"value\":\"1\"}",
                               "release_control": "!{\"cmd\":\"drone\",\"param\":\"control_app\",\"value\":\"0\"}"},
              "mission_cmds_verified": False},
    "mediamtx": {"base_url": "http://localhost:8889", "record": True, "record_path": "./recordings/%path/%Y-%m-%d_%H-%M-%S-%f"},
    "trackers": {}
}

COT_TYPES = [("Friendly Air", "a-f-A"), ("Friendly Air UAS (H-q)", "a-f-A-C-H-q"), ("Friendly Air Mil UAS", "a-f-A-M-F-Q"),
             ("Hostile Air", "a-h-A"), ("Hostile Air UAS (H-q)", "a-h-A-C-H-q"), ("Neutral Air", "a-n-A"),
             ("Unknown Air", "a-u-A"), ("Friendly Ground", "a-f-G"), ("Hostile Ground", "a-h-G")]

# ------------------------------------------------------------------------- helpers
def nmea_checksum_ok(s):
    if not s.startswith("$"): return False
    body, star, cks = s[1:].partition("*")
    if not star: return False
    if cks == "": return True
    x = 0
    for ch in body: x ^= ord(ch)
    try: return x == int(cks[:2], 16)
    except ValueError: return False

def dm_to_deg(v, hemi):
    if not v: return None
    try: f = float(v)
    except ValueError: return None
    d = int(f // 100); deg = d + (f - d * 100) / 60.0
    return -deg if hemi in ("S", "W") else deg

def baro_altitude(p_pa, t_c, p0_pa):
    if p_pa <= 0: return None
    return ((p0_pa / p_pa) ** (1 / 5.257) - 1.0) * (t_c + 273.15) / 0.0065

def sea_level_pressure(p_pa, t_c, h_m):
    return p_pa * (1.0 - (0.0065 * h_m) / (t_c + 273.15 + 0.0065 * h_m)) ** -5.257

def iso_z(t=None):
    t = t or dt.datetime.now(dt.timezone.utc)
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (t.microsecond // 1000)

def deep_update(base, new):
    for k, v in new.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict): deep_update(base[k], v)
        else: base[k] = v
    return base

def is_multicast(host):
    try: return 224 <= int(host.split(".")[0]) <= 239
    except ValueError: return False

# ------------------------------------------------------------------------- state
class Track:
    def __init__(self, tid):
        self.id = tid; self.callsign = None
        self.fix_valid = 0; self.lat = self.lon = None; self.gps_alt = None; self.hdop = None; self.nsat = None
        self.sog = None; self.cog = None; self.geoid_sep = None
        self.pressure_pa = None; self.temp_c = None; self.baro_alt_std = None; self.baro_alt = None
        self.rssi = None; self.batt_mv = None; self.freq_code = None
        self.gps_time = None; self.rx_time = None; self.msg_count = 0; self.last_cot = 0.0
        self.trail = collections.deque(maxlen=2000)      # (t, lat, lon, alt)
        self.rx_hist = collections.deque(maxlen=600)     # (t, slot, rssi)

    def age(self):
        return (time.time() - self.rx_time.timestamp()) if self.rx_time else None

    def to_dict(self):
        d = {k: v for k, v in self.__dict__.items() if k not in ("trail", "rx_hist")}
        d["gps_time"] = self.gps_time.isoformat() if self.gps_time else None
        d["rx_time"] = self.rx_time.isoformat() if self.rx_time else None
        d["age_s"] = round(self.age(), 1) if self.rx_time else None
        return d

class ReceiverParser:
    def __init__(self, on_report, on_raw):
        self.on_report = on_report; self.on_raw = on_raw; self.cur = None; self.bad = 0; self.sentences = 0

    def feed_line(self, line):
        line = line.strip()
        if not line: return
        self.sentences += 1
        self.on_raw(line)
        if not nmea_checksum_ok(line):
            self.bad += 1; return
        f = line[1:].split("*", 1)[0].split(","); tag = f[0]
        if tag == "RFMSGFROM":
            self.cur = {"raw": [line]}
            try:
                code = f[1]; self.cur["freq_code"] = code[:-3] if len(code) > 3 else None; self.cur["addr"] = int(code[-3:])
                self.cur["len_hex"] = f[2] if len(f) > 2 else None
                self.cur["dest"] = int(f[3]) if len(f) > 3 and f[3].isdigit() else None
            except (IndexError, ValueError): self.cur = None
            return
        if self.cur is None: return
        self.cur["raw"].append(line)
        try:
            if tag == "HRFSSI": self.cur["rssi"] = float(f[1])
            elif tag == "BATMV": self.cur["batt_mv"] = float(f[1])
            elif tag == "BAROALT":
                self.cur["pressure_pa"] = float(f[1]); self.cur["temp_c"] = float(f[2]); self.cur["baro_alt_std"] = float(f[3])
            elif tag in ("GPGGA", "GNGGA"):
                self.cur["utc"] = f[1]; self.cur["lat"] = dm_to_deg(f[2], f[3]); self.cur["lon"] = dm_to_deg(f[4], f[5])
                self.cur["fix"] = int(f[6] or 0); self.cur["nsat"] = int(f[7] or 0)
                self.cur["hdop"] = float(f[8]) if f[8] else None; self.cur["alt"] = float(f[9]) if f[9] else None
                self.cur["geoid"] = float(f[11]) if len(f) > 11 and f[11] else None
            elif tag in ("GPRMC", "GNRMC"):
                self.cur["utc"] = f[1] or self.cur.get("utc"); self.cur["rmc_valid"] = (f[2] == "A")
                if self.cur.get("lat") is None:
                    self.cur["lat"] = dm_to_deg(f[3], f[4]); self.cur["lon"] = dm_to_deg(f[5], f[6])
                self.cur["sog_ms"] = float(f[7]) * 0.514444 if f[7] else None
                self.cur["cog"] = float(f[8]) if f[8] else None; self.cur["date"] = f[9]
            elif tag == "RFMSGEND":
                rep = self.cur; self.cur = None; self.on_report(rep)
        except (IndexError, ValueError): self.bad += 1

# ------------------------------------------------------------------------- CoT
def video_xml(url, alias):
    return ('<__video uid="%s" url="%s"><ConnectionEntry networkTimeout="12000" uid="%s" path="" protocol="rtsp" bufferTime="-1" '
            'address="%s" port="554" roverPort="-1" rtspReliable="0" ignoreEmbeddedKLV="false" alias="%s"/></__video>'
            ) % (alias, url, alias, url.split("//")[-1].split(":")[0].split("/")[0], alias)

def cot_xml(uid, typ, callsign, lat, lon, hae, ce, speed, course, altsrc, remarks, stale_s, now, flow_tag=False, video=None, extra=""):
    stale = now + dt.timedelta(seconds=float(stale_s))
    flow = '<_flow-tags_ sitgs="%s"/>' % iso_z(now) if flow_tag else ""
    vid = video_xml(video[0], video[1]) if video else ""
    return ('<?xml version="1.0"?><event version="2.0" uid="%s" type="%s" how="m-g" time="%s" start="%s" stale="%s">'
            '<point lat="%.7f" lon="%.7f" hae="%.1f" ce="%.1f" le="9999999"/><detail><contact callsign="%s"/>'
            '<track speed="%.2f" course="%.1f"/><precisionlocation geopointsrc="GPS" altsrc="%s"/><remarks>%s</remarks>%s%s%s</detail></event>'
            ) % (uid, typ, iso_z(now), iso_z(now), iso_z(stale), lat, lon, hae, ce, callsign, speed or 0.0, course or 0.0, altsrc, remarks, vid, extra, flow)

def cot_event(tr, cfg, now, video=None, source="LoRa915"):
    c = cfg["cot"]; tcfg = cfg["trackers"].get(str(tr.id), {})
    uid = c["uid_prefix"] + (c["uid_format"] % tr.id)
    typ = tcfg.get("cot_type") or c["type"]; callsign = tcfg.get("callsign") or uid
    geoid = tr.geoid_sep if (c["geoid_auto_update"] and tr.geoid_sep is not None) else float(c["geoid_height_m"])
    use_baro = cfg["barometer"]["mode"] != "disable" and tr.baro_alt is not None
    alt_msl = tr.baro_alt if use_baro else tr.gps_alt
    hae = (alt_msl if alt_msl is not None else 0.0) + geoid
    ce = (tr.hdop or 1.0) * 2.5
    remarks = "id=%d rssi=%s batt=%smV baro=%s src=%s" % (tr.id, tr.rssi, tr.batt_mv, None if tr.baro_alt is None else round(tr.baro_alt, 1), source)
    return cot_xml(uid, typ, callsign, tr.lat, tr.lon, hae, ce, tr.sog, tr.cog, "BARO" if use_baro else "GPS", remarks,
                   c["stale_s"], now, c["include_flow_tag"], video)

def cot_event_dji(box, cfg, now, tracker=None, geoid_default=None):
    """CoT built from a Cosmostreamer box; if paired, carries the tracker's uid/callsign so the drone is one entity."""
    c = cfg["cot"]; co = cfg["cosmo"]
    if tracker is not None:
        tcfg = cfg["trackers"].get(str(tracker.id), {})
        uid = c["uid_prefix"] + (c["uid_format"] % tracker.id); callsign = tcfg.get("callsign") or uid; typ = tcfg.get("cot_type") or c["type"]
        geoid = tracker.geoid_sep if (c["geoid_auto_update"] and tracker.geoid_sep is not None) else float(c["geoid_height_m"])
    else:
        uid = co.get("unpaired_uid_prefix", "DJI.") + (box.name or box.key).replace(" ", "_"); callsign = box.name or uid; typ = c["type"]
        geoid = float(c["geoid_height_m"])
    alt_rel = box.alt_rel() or 0.0
    msl = (box.takeoff_msl if box.takeoff_msl is not None else 0.0) + alt_rel
    remarks = "dji=%s model=%s sats=%s batt=%s%% mode=%s alt_rel=%.0f src=DJI" % (box.name, box.values.get("camera_model"), box.values.get("gps_sat_count"),
                                                                                   box.values.get("battery_level"), box.values.get("flight_mode"), alt_rel)
    video = (box.to_dict()["video_url"], box.name or box.key) if (co.get("video_in_cot", True) and box.transport == "udp") else None
    return cot_xml(uid, typ, callsign, box.lat(), box.lng(), msl + geoid, 3.0, box.hspeed(), box.yaw(), "GPS", remarks,
                   c["stale_s"], now, c["include_flow_tag"], video)

class CotSender:
    def __init__(self, dests, status):
        self.socks = []; self.sent = 0; self.errors = 0
        for i, d in enumerate(dests):
            if not d.get("enabled", True): continue
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                if is_multicast(d["host"]):
                    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, struct.pack("b", int(d.get("ttl", 2))))
                    iface = d.get("iface", "0.0.0.0")
                    if iface and iface != "0.0.0.0": s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(iface))
                elif d.get("iface") and d["iface"] != "0.0.0.0":
                    s.bind((d["iface"], 0))
                self.socks.append((s, (d["host"], int(d["port"]))))
                status("Initializing sink for ID=%d (%s to %s:%s, TTL %s)" % (i + 1, "Multicast" if is_multicast(d["host"]) else "Auto", d["host"], d["port"], d.get("ttl", 2)))
            except OSError as e:
                status("Sink ID=%d FAILED (%s:%s): %s" % (i + 1, d.get("host"), d.get("port"), e))

    def send(self, xml, status):
        for s, addr in self.socks:
            try: s.sendto(xml.encode(), addr); self.sent += 1
            except OSError as e:
                self.errors += 1
                if self.errors <= 5: status("send error to %s:%s: %s" % (addr[0], addr[1], e))

    def close(self):
        for s, _ in self.socks: s.close()

# ------------------------------------------------------------------------- ground station
class GroundStation:
    CSV_COLS = ["Measurement_DateTime", "Measurement_ReceivedDateTime", "GPS_FixValid", "GPS_lat", "GPS_lon", "GPS_alt",
                "GPS_HDOP", "GPS_SOG", "GPS_COG", "Barometer_Pressure", "Barometer_Temperature", "Barometer_Altitude",
                "Report_StationID", "RF_RSSI"]

    def __init__(self, cfg, cfg_path):
        self.cfg = cfg; self.cfg_path = cfg_path
        self.tracks = {}; self.lock = threading.Lock()
        self.parser = ReceiverParser(self.on_report, self.on_raw)
        self.raw = collections.deque(maxlen=300); self.cot_status = collections.deque(maxlen=50)
        self.debug = collections.deque(maxlen=200)
        self.start = time.time(); self.lines = 0; self.bytes = 0
        self.conn_status = "disconnected"; self.connected = False; self.conn_thread = None; self.conn_stop = threading.Event()
        self.nmea_fh = None; self.nmea_path = None; self.nmea_lines = 0
        self.csv_fhs = {}; self.csv_rows = 0; self.session_stamp = None
        self.cot = None; self.rebuild_cot()
        self.reports = 0; self.ghosts = 0; self.sea_level_pa = float(cfg["barometer"]["ref_pressure_bar"]) * 1e5
        self.replay = None
        self.apply_logging()
        self.cosmo = CosmoClient(lambda: self.cfg["cosmo"], self.dbg); self.cosmo.start()
        self.missions = mission_mod.MissionStore(os.path.join(DATA_DIR, "logs", "missions"))
        self.drone_cot_sent = 0
        threading.Thread(target=self.kml_loop, daemon=True).start()
        threading.Thread(target=self.fusion_loop, daemon=True).start()

    # ---- status helpers
    def cot_log(self, msg): self.cot_status.append("%s  %s" % (dt.datetime.now().strftime("%H:%M:%S"), msg))
    def dbg(self, msg): self.debug.append("%s  %s" % (dt.datetime.now().strftime("%H:%M:%S.%f")[:-3], msg))

    # ---- config
    def save_config(self):
        with open(self.cfg_path, "w") as f: json.dump(self.cfg, f, indent=2)

    def update_config(self, patch):
        with self.lock:
            old_cot = json.dumps(self.cfg["cot"]["destinations"]); old_log = json.dumps(self.cfg["log"])
            deep_update(self.cfg, patch)
            if "trackers" in patch: self.cfg["trackers"] = patch["trackers"]
            if "destinations" in patch.get("cot", {}): self.cfg["cot"]["destinations"] = patch["cot"]["destinations"]
            if "pairs" in patch.get("cosmo", {}): self.cfg["cosmo"]["pairs"] = patch["cosmo"]["pairs"]
            if "manual_hosts" in patch.get("cosmo", {}): self.cfg["cosmo"]["manual_hosts"] = patch["cosmo"]["manual_hosts"]
            for tr in self.tracks.values(): tr.callsign = self.cfg["trackers"].get(str(tr.id), {}).get("callsign") or self.uid(tr.id)
            self.save_config()
        if json.dumps(self.cfg["cot"]["destinations"]) != old_cot: self.rebuild_cot()
        if json.dumps(self.cfg["log"]) != old_log: self.apply_logging()

    def uid(self, tid):
        try: return self.cfg["cot"]["uid_prefix"] + (self.cfg["cot"]["uid_format"] % tid)
        except (TypeError, ValueError): return "%s%d" % (self.cfg["cot"]["uid_prefix"], tid)

    def rebuild_cot(self):
        if self.cot: self.cot.close()
        self.cot = CotSender(self.cfg["cot"]["destinations"], self.cot_log)

    # ---- logging
    def apply_logging(self):
        L = self.cfg["log"]
        if self.session_stamp is None: self.session_stamp = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        if L["nmea_enabled"] and self.nmea_fh is None:
            d = os.path.join(DATA_DIR, L["nmea_dir"]); os.makedirs(d, exist_ok=True)
            self.nmea_path = os.path.join(d, "%s_RFReceiverRSSILog.nmea" % self.session_stamp)
            self.nmea_fh = open(self.nmea_path, "a", buffering=1); self.dbg("NMEA log opened %s" % self.nmea_path)
        elif not L["nmea_enabled"] and self.nmea_fh is not None:
            self.nmea_fh.close(); self.nmea_fh = None; self.dbg("NMEA log closed")
        if not L["split_enabled"] and self.csv_fhs:
            for f, _ in self.csv_fhs.values(): f.close()
            self.csv_fhs = {}; self.dbg("split CSV logs closed")

    def on_raw(self, line):
        self.lines += 1; self.raw.append(line)
        if self.nmea_fh:
            if line.startswith("$RFMSGEND") and self.cfg["log"]["inject_rx_timestamp"]:
                self.nmea_fh.write("$RXTIMESTAMP,%s*\n" % dt.datetime.now(dt.timezone.utc).strftime("%Y,%m,%d,%H,%M,%S.%f")[:-3]); self.nmea_lines += 1
            self.nmea_fh.write(line + "\n"); self.nmea_lines += 1

    def on_report(self, r):
        # drop noise/corrupt packets: wrong destination address or no payload sentences (RSSI near the noise floor)
        flt = self.cfg.get("filter", {})
        ra = flt.get("receiver_address")
        if (ra not in (None, "", 0) and r.get("dest") is not None and r["dest"] != int(ra)) or \
           (flt.get("require_payload", True) and r.get("utc") is None and r.get("pressure_pa") is None):
            self.ghosts += 1
            if self.ghosts <= 20: self.dbg("rejected packet: hdr=%s rssi=%s" % (r["raw"][0], r.get("rssi")))
            return
        now = dt.datetime.now(dt.timezone.utc); tid = r["addr"]; self.reports += 1
        with self.lock:
            tr = self.tracks.get(tid)
            if tr is None:
                tr = self.tracks[tid] = Track(tid); tr.callsign = self.cfg["trackers"].get(str(tid), {}).get("callsign") or self.uid(tid)
                self.dbg("new tracker id=%d freq_code=%s" % (tid, r.get("freq_code")))
            tr.msg_count += 1; tr.rx_time = now; tr.freq_code = r.get("freq_code")
            for ks, kd in (("rssi", "rssi"), ("batt_mv", "batt_mv"), ("pressure_pa", "pressure_pa"), ("temp_c", "temp_c"),
                           ("baro_alt_std", "baro_alt_std"), ("hdop", "hdop"), ("nsat", "nsat"), ("alt", "gps_alt"),
                           ("geoid", "geoid_sep"), ("sog_ms", "sog"), ("cog", "cog")):
                if r.get(ks) is not None: setattr(tr, kd, r[ks])
            fix = r.get("fix", 0)
            if fix and r.get("lat") is not None: tr.fix_valid = fix; tr.lat = r["lat"]; tr.lon = r["lon"]
            else: tr.fix_valid = 0
            if r.get("utc") and r.get("date") and len(r["date"]) == 6:
                try:
                    hh, mm, ss = int(r["utc"][0:2]), int(r["utc"][2:4]), float(r["utc"][4:])
                    d, m, y = int(r["date"][0:2]), int(r["date"][2:4]), 2000 + int(r["date"][4:6])
                    tr.gps_time = dt.datetime(y, m, d, hh, mm, int(ss), int((ss % 1) * 1e6), tzinfo=dt.timezone.utc)
                except ValueError: pass
            t = now.timestamp(); slot = int((t % 1.0) * 1000 // int(self.cfg["tdma_slot_ms"]))
            tr.rx_hist.append((t, slot, tr.rssi))
            self.apply_barometer(tr)
            if tr.fix_valid: tr.trail.append((t, tr.lat, tr.lon, tr.gps_alt))
        if tr.fix_valid:
            self.write_csv(tr, now); self.maybe_send_cot(tr, now)

    def apply_barometer(self, tr):
        b = self.cfg["barometer"]
        if tr.pressure_pa is None: tr.baro_alt = None; return
        if b["auto_update"] == "tracker" and int(b["ref_tracker_id"] or 0) == tr.id and tr.temp_c is not None:
            self.sea_level_pa = sea_level_pressure(tr.pressure_pa, tr.temp_c, float(b["ref_altitude_m"]))
            b["ref_pressure_bar"] = round(self.sea_level_pa / 1e5, 5); b["ref_temp_c"] = tr.temp_c
        elif b["auto_update"] != "tracker":
            self.sea_level_pa = sea_level_pressure(float(b["ref_pressure_bar"]) * 1e5, float(b["ref_temp_c"]), float(b["ref_altitude_m"]))
        if b["mode"] == "recompute": tr.baro_alt = baro_altitude(tr.pressure_pa, float(b["ref_temp_c"]), self.sea_level_pa)
        else: tr.baro_alt = tr.baro_alt_std

    def write_csv(self, tr, now):
        L = self.cfg["log"]
        if not L["split_enabled"]: return
        fh = self.csv_fhs.get(tr.id)
        if fh is None:
            d = os.path.join(DATA_DIR, L["split_dir"]); os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "%s_%s_ID%03d.csv" % (self.session_stamp, L["event_name"], tr.id)); new = not os.path.exists(path)
            f = open(path, "a", newline="", buffering=1); w = csv.writer(f)
            if new: w.writerow(self.CSV_COLS)
            fh = self.csv_fhs[tr.id] = (f, w); self.dbg("split CSV opened %s" % path)
        _, w = fh; b = self.cfg["barometer"]["mode"]
        w.writerow([tr.gps_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] if tr.gps_time else "",
                    now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], tr.fix_valid, "%.6f" % tr.lat, "%.6f" % tr.lon,
                    "" if tr.gps_alt is None else "%.1f" % tr.gps_alt, "" if tr.hdop is None else tr.hdop,
                    "" if tr.sog is None else "%.2f" % tr.sog, "" if tr.cog is None else "%.1f" % tr.cog,
                    "" if tr.pressure_pa is None else "%.5f" % (tr.pressure_pa / 1e5), "" if tr.temp_c is None else tr.temp_c,
                    "" if (tr.baro_alt is None or b == "disable") else "%.1f" % tr.baro_alt, "%d.0" % tr.id,
                    "" if tr.rssi is None else tr.rssi])
        self.csv_rows += 1

    # ---- Cosmostreamer pairing / fusion
    def pair_for_tracker(self, tid):
        for key, t in self.cfg["cosmo"].get("pairs", {}).items():
            try:
                if int(t) == int(tid): return self.cosmo.box(key)
            except (TypeError, ValueError): pass
        return None

    def dji_live(self, box):
        return box is not None and box.connected() and box.has_fix() and (time.time() - box.last_telemetry) < float(self.cfg["cosmo"].get("dji_stale_s", 2.5))

    def maybe_send_cot(self, tr, now):
        if not self.cfg["cot"]["enabled"]: return
        if (time.time() - tr.last_cot) * 1000.0 < float(self.cfg["cot"]["period_ms"]): return
        box = self.pair_for_tracker(tr.id)
        if box is not None:
            # keep the take-off MSL current while the drone is on the ground next to its tracker
            if box.values.get("in_flight") == "0" and tr.gps_alt is not None: box.takeoff_msl = tr.gps_alt
            elif box.takeoff_msl is None and tr.gps_alt is not None and box.alt_rel() is not None: box.takeoff_msl = tr.gps_alt - box.alt_rel()
            if self.dji_live(box): return   # the fusion loop is emitting this drone from the DJI link
        tr.last_cot = time.time()
        video = (box.to_dict()["video_url"], box.name or box.key) if (box is not None and self.cfg["cosmo"].get("video_in_cot", True) and box.transport == "udp") else None
        self.cot.send(cot_event(tr, self.cfg, now, video=video, source="LoRa915" + ("(fallback)" if box is not None else "")), self.cot_log)

    def fusion_loop(self):
        while True:
            try:
                period = max(0.2, float(self.cfg["cot"]["period_ms"]) / 1000.0)
                time.sleep(period)
                if not self.cfg["cot"]["enabled"] or not self.cfg["cosmo"].get("enabled"): continue
                now = dt.datetime.now(dt.timezone.utc); pairs = self.cfg["cosmo"].get("pairs", {})
                for b in list(self.cosmo.boxes.values()):
                    if not self.dji_live(b): continue
                    tid = pairs.get(b.key)
                    tr = None
                    if tid not in (None, "", 0):
                        with self.lock: tr = self.tracks.get(int(tid))
                        if tr is None:
                            tr = Track(int(tid)); tr.callsign = self.cfg["trackers"].get(str(tid), {}).get("callsign") or self.uid(int(tid))
                    elif not self.cfg["cosmo"].get("cot_unpaired", True): continue
                    self.cot.send(cot_event_dji(b, self.cfg, now, tr), self.cot_log); self.drone_cot_sent += 1
            except Exception as e:
                self.dbg("fusion error: %s" % e); time.sleep(1)

    # ---- KML / GeoJSON
    def kml_loop(self):
        while True:
            try:
                if self.cfg["kml"]["enabled"]: self.write_kml()
            except Exception as e: self.dbg("KML error: %s" % e)
            time.sleep(max(0.5, float(self.cfg["kml"]["interval_s"])))

    def build_kml(self):
        tol = float(self.cfg["data_age_tolerance_s"]); n = int(self.cfg["kml"]["trail_points"]); out = []
        with self.lock:
            for tr in self.tracks.values():
                if tr.lat is None or (tr.age() or 1e9) > tol: continue
                name = tr.callsign or self.uid(tr.id); alt = tr.gps_alt or 0
                coords = " ".join("%.6f,%.6f,%.1f" % (lon, lat, a or 0) for _, lat, lon, a in list(tr.trail)[-n:])
                out.append('<Placemark><name>%s</name><description>id=%d rssi=%s batt=%s mV age=%.0fs</description>'
                           '<Point><altitudeMode>absolute</altitudeMode><coordinates>%.6f,%.6f,%.1f</coordinates></Point></Placemark>'
                           '<Placemark><name>%s trail</name><Style><LineStyle><color>ff33ccff</color><width>2</width></LineStyle></Style>'
                           '<LineString><altitudeMode>absolute</altitudeMode><coordinates>%s</coordinates></LineString></Placemark>'
                           % (name, tr.id, tr.rssi, tr.batt_mv, tr.age() or 0, tr.lon, tr.lat, alt, name, coords))
        return '<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>sitgs</name>%s</Document></kml>' % "".join(out)

    def build_geojson(self):
        tol = float(self.cfg["data_age_tolerance_s"]); feats = []
        with self.lock:
            for tr in self.tracks.values():
                if tr.lat is None or (tr.age() or 1e9) > tol: continue
                feats.append({"type": "Feature", "geometry": {"type": "Point", "coordinates": [tr.lon, tr.lat, tr.gps_alt or 0]},
                              "properties": {"id": tr.id, "callsign": tr.callsign, "rssi": tr.rssi, "batt_mv": tr.batt_mv, "age_s": tr.age(),
                                             "sog": tr.sog, "cog": tr.cog, "baro_alt": tr.baro_alt}})
        return json.dumps({"type": "FeatureCollection", "features": feats})

    def write_kml(self):
        k = self.cfg["kml"]
        for path, data in ((k["path"], self.build_kml()), (k.get("geojson_path"), self.build_geojson())):
            if not path: continue
            p = os.path.join(DATA_DIR, path); os.makedirs(os.path.dirname(p), exist_ok=True)
            tmp = p + ".tmp"
            with open(tmp, "w") as f: f.write(data)
            os.replace(tmp, p)

    # ---- connection
    def connect(self):
        self.disconnect()
        c = self.cfg["connection"]; self.conn_stop.clear()
        self.conn_thread = threading.Thread(target=self.conn_loop, args=(dict(c),), daemon=True); self.conn_thread.start()

    def disconnect(self):
        self.conn_stop.set()
        if self.conn_thread and self.conn_thread.is_alive(): self.conn_thread.join(timeout=3)
        self.conn_thread = None; self.connected = False
        if self.conn_status != "disconnected": self.conn_status = "disconnected"

    def conn_loop(self, c):
        if serial is None:
            self.conn_status = "pyserial not available"; return
        if c["type"] == "network": port = "socket://%s:%s" % (c["host"], c["tcp_port"])
        else:
            port = c["port"]
            if port == "auto":
                port = find_port()
                if not port: self.conn_status = "no serial port found"; return
        rec = open(os.path.join(DATA_DIR, c["record_raw"]), "ab") if c.get("record_raw") else None
        while not self.conn_stop.is_set():
            try:
                self.conn_status = "opening %s" % port
                with serial.serial_for_url(port, baudrate=int(c["baud"]), timeout=0.5) as ser:
                    self.conn_status = "connected %s" % port; self.connected = True; self.dbg("connected %s" % port); buf = b""
                    while not self.conn_stop.is_set():
                        chunk = ser.read(getattr(ser, "in_waiting", 0) or 1)
                        if not chunk: continue
                        self.bytes += len(chunk)
                        if rec: rec.write(chunk); rec.flush()
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1); self.parser.feed_line(line.decode("ascii", "replace"))
            except Exception as e:
                self.connected = False; self.conn_status = "error: %s (retrying)" % str(e)[:80]; self.dbg("connection error: %s" % e)
                self.conn_stop.wait(2)
        self.connected = False; self.conn_status = "disconnected"
        if rec: rec.close()

    def replay_file(self, path, speed):
        def run():
            self.conn_status = "replaying %s" % os.path.basename(path); self.connected = True
            with open(path, "rb") as f:
                for raw in f:
                    if self.conn_stop.is_set(): break
                    line = raw.decode("ascii", "replace").strip(); self.parser.feed_line(line)
                    if line.startswith("$RFMSGEND") and speed > 0: time.sleep(1.0 / speed)
            self.conn_status = "replay finished"; self.connected = False
        self.disconnect(); self.conn_stop.clear()
        self.conn_thread = threading.Thread(target=run, daemon=True); self.conn_thread.start()

    # ---- snapshot for UI
    def snapshot(self):
        with self.lock:
            tol = float(self.cfg["data_age_tolerance_s"]); cutoff = time.time() - 120
            timeline = {str(t.id): [[round(x - cutoff, 2), s, r] for x, s, r in t.rx_hist if x >= cutoff] for t in self.tracks.values()}
            for b in self.cosmo.boxes.values():
                pts = [[round(x - cutoff, 2), None, None] for x, _, _ in b.rx_hist if x >= cutoff]
                if pts or b.online: timeline["DJI " + (b.name or b.key)] = pts
            cs = self.cosmo.snapshot(); pairs = self.cfg["cosmo"].get("pairs", {})
            for bd in cs["boxes"]:
                box = self.cosmo.boxes.get(bd["key"]); bd["tracker_id"] = pairs.get(bd["key"]) or None
                bd["source"] = "DJI" if self.dji_live(box) else ("LoRa" if bd["tracker_id"] else "-")
                bd["trail"] = [[la, lo] for _, la, lo, _ in list(box.trail)[-500:]] if box else []
            cs["pairs"] = pairs; cs["drone_cot_sent"] = self.drone_cot_sent
            return {"uptime_s": round(time.time() - self.start, 1), "conn_status": self.conn_status, "connected": self.connected,
                    "lines": self.lines, "bytes": self.bytes, "reports": self.reports, "bad_sentences": self.parser.bad, "ghosts": self.ghosts,
                    "cot_sent": self.cot.sent, "cot_errors": self.cot.errors, "cot_status": list(self.cot_status),
                    "nmea_path": self.nmea_path if self.nmea_fh else None, "nmea_lines": self.nmea_lines, "csv_rows": self.csv_rows,
                    "csv_files": {str(k): os.path.basename(f.name) for k, (f, _) in self.csv_fhs.items()},
                    "sea_level_bar": round(self.sea_level_pa / 1e5, 5), "data_age_tolerance_s": tol,
                    "raw": list(self.raw)[-120:], "debug": list(self.debug)[-100:], "timeline_window_s": 120, "timeline": timeline,
                    "cosmo": cs,
                    "trackers": [dict(t.to_dict(), paired_box=next((k for k, v in pairs.items() if str(v) == str(t.id)), None)) for t in sorted(self.tracks.values(), key=lambda t: t.id)],
                    "trails": {str(t.id): [[lat, lon] for _, lat, lon, _ in list(t.trail)[-500:]] for t in self.tracks.values()}}

def find_port(probe=True):
    """Pick the receiver. Pucks plugged in over USB are also Feather M0s, so when more than one
    candidate exists each is opened briefly and the one emitting $RFMSGFROM / $RF (receiver framing) wins."""
    if serial is None: return None
    cands = []
    for p in serial.tools.list_ports.comports():
        score = 0
        if p.vid == 0x239A: score += 10
        if "usbmodem" in p.device or "ttyACM" in p.device or "ttyUSB" in p.device: score += 3
        if p.description and ("Feather" in p.description or "Adafruit" in p.description): score += 5
        if score: cands.append((score, p.device))
    cands.sort(reverse=True)
    if not cands: return None
    if len(cands) == 1 or not probe: return cands[0][1]
    for _, dev in cands:
        try:
            with serial.Serial(dev, 115200, timeout=0.2) as s:
                t0 = time.time(); buf = b""
                while time.time() - t0 < 2.5:
                    buf += s.read(256)
                    if b"$RFMSGFROM" in buf or b"$RF," in buf or b"$HRFSSI" in buf: return dev
                    if b"$GPGSV" in buf or b"$LOGGERID" in buf or b"$BAROALT," in buf and b"$RFMSGFROM" not in buf: break  # a puck, not the receiver
        except (serial.SerialException, OSError):
            continue
    return cands[0][1]

def list_ports():
    if serial is None: return []
    return [{"device": p.device, "description": p.description or "", "vid": p.vid, "pid": p.pid} for p in serial.tools.list_ports.comports()]

# ------------------------------------------------------------------------- web
def make_handler(gs, page):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, code, ctype, body):
            if isinstance(body, str): body = body.encode()
            self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/api/state": self._send(200, "application/json", json.dumps(gs.snapshot()))
            elif p == "/api/config": self._send(200, "application/json", json.dumps({"config": gs.cfg, "cot_types": COT_TYPES, "here": DATA_DIR}))
            elif p == "/api/ports": self._send(200, "application/json", json.dumps({"ports": list_ports(), "auto": find_port(probe=False)}))
            elif p == "/api/missions": self._send(200, "application/json", json.dumps({"missions": gs.missions.list(), "drone_enums": mission_mod.DRONE_ENUMS, "finish_actions": mission_mod.FINISH_ACTIONS}))
            elif p.startswith("/missions/") and p.endswith(".kmz"):
                m = gs.missions.get(p[10:-4])
                if m: self._send(200, "application/vnd.google-earth.kmz", mission_mod.build_kmz(m))
                else: self._send(404, "text/plain", "no such mission")
            elif p == "/mediamtx.yml":
                mm = gs.cfg["mediamtx"]; self._send(200, "text/yaml", mission_mod.mediamtx_yaml(gs.cosmo.snapshot()["boxes"], mm.get("record", True), mm.get("record_path")))
            elif p == "/live.kml": self._send(200, "application/vnd.google-earth.kml+xml", gs.build_kml())
            elif p == "/live.geojson": self._send(200, "application/geo+json", gs.build_geojson())
            elif p == "/link.kml":
                self._send(200, "application/vnd.google-earth.kml+xml",
                           '<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2"><NetworkLink><name>sitgs live</name>'
                           '<Link><href>http://%s/live.kml</href><refreshMode>onInterval</refreshMode><refreshInterval>%s</refreshInterval></Link></NetworkLink></kml>'
                           % (self.headers.get("Host", "localhost"), gs.cfg["kml"]["interval_s"]))
            elif p.startswith("/static/") and ".." not in p:
                fp = os.path.join(RES_DIR, p.lstrip("/"))
                if os.path.isfile(fp):
                    ct = {"js": "application/javascript", "css": "text/css", "png": "image/png"}.get(fp.rsplit(".", 1)[-1], "application/octet-stream")
                    with open(fp, "rb") as f: self._send(200, ct, f.read())
                else: self._send(404, "text/plain", "not found")
            else: self._send(200, "text/html; charset=utf-8", page)
        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0)); body = json.loads(self.rfile.read(n) or b"{}"); p = self.path
            try:
                if p == "/api/config": gs.update_config(body); self._send(200, "application/json", json.dumps({"ok": True, "config": gs.cfg}))
                elif p == "/api/connect":
                    if body: gs.update_config({"connection": body})
                    gs.connect(); self._send(200, "application/json", '{"ok":true}')
                elif p == "/api/disconnect": gs.disconnect(); self._send(200, "application/json", '{"ok":true}')
                elif p == "/api/replay": gs.replay_file(os.path.join(DATA_DIR, body["path"]), float(body.get("speed", 1))); self._send(200, "application/json", '{"ok":true}')
                elif p == "/api/mission/save": m = gs.missions.save(body); self._send(200, "application/json", json.dumps({"ok": True, "mission": m, "stats": mission_mod.mission_stats(m)}))
                elif p == "/api/mission/delete": gs.missions.delete(body["name"]); self._send(200, "application/json", '{"ok":true}')
                elif p == "/api/mission/upload":
                    m = gs.missions.get(body["name"]); box = gs.cosmo.box(body["box_key"])
                    if not m or not box: raise ValueError("unknown mission or box")
                    kmz = mission_mod.build_kmz(m)
                    if box.transport == "udp":
                        res = mission_mod.upload_to_box(box.host, m["name"], kmz)
                    else:
                        raise ValueError("upload over MQTT not wired: publish %s to cosmo/<board_id>/control" % "wp file_upload")
                    gs.dbg("mission %s uploaded to %s: %s" % (m["name"], box.name, res)); self._send(200, "application/json", json.dumps({"ok": res == "ok", "result": res}))
                elif p == "/api/mission/cmd":
                    box = gs.cosmo.box(body["box_key"]); act = body["action"]; cmds = gs.cfg["cosmo"]["mission_cmds"]
                    if not box or act not in cmds: raise ValueError("unknown box or action")
                    cmd = cmds[act].replace("%NAME%", mission_mod.safe_name(body.get("name", "")))
                    gs.cosmo.send_text_command(box.key, cmd); gs.dbg("sent to %s: %s" % (box.name, cmd)); gs.cot_log("drone cmd -> %s: %s" % (box.name, act))
                    self._send(200, "application/json", json.dumps({"ok": True, "sent": cmd}))
                elif p == "/api/clear":
                    with gs.lock: gs.tracks.clear()
                    self._send(200, "application/json", '{"ok":true}')
                else: self._send(404, "text/plain", "no")
            except Exception as e:
                self._send(500, "application/json", json.dumps({"ok": False, "error": str(e)}))
    return H

def main():
    ap = argparse.ArgumentParser(description="SIT GPS-LoRa tracker ground station")
    ap.add_argument("--config", default=os.path.join(DATA_DIR, "sitgs.json"))
    ap.add_argument("--port"); ap.add_argument("--baud", type=int); ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--record", help="append raw serial bytes to this file"); ap.add_argument("--replay"); ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--web-port", type=int); ap.add_argument("--no-connect", action="store_true")
    args = ap.parse_args()
    if args.list_ports:
        for p in list_ports(): print("%-30s vid=%s pid=%s  %s" % (p["device"], hex(p["vid"]) if p["vid"] else "-", hex(p["pid"]) if p["pid"] else "-", p["description"]))
        print("autodetect ->", find_port()); return 0
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if os.path.exists(args.config):
        with open(args.config) as f: deep_update(cfg, json.load(f))
    if args.port: cfg["connection"]["port"] = args.port; cfg["connection"]["type"] = "serial"
    if args.baud: cfg["connection"]["baud"] = args.baud
    if args.record: cfg["connection"]["record_raw"] = args.record
    if args.web_port: cfg["web"]["port"] = args.web_port
    with open(args.config, "w") as f: json.dump(cfg, f, indent=2)
    gs = GroundStation(cfg, args.config)
    page = open(os.path.join(RES_DIR, "sitgs_ui.html"), "rb").read()
    port = int(cfg["web"]["port"])
    try: srv = ThreadingHTTPServer(("0.0.0.0", port), make_handler(gs, page))
    except OSError:
        port += 1; srv = ThreadingHTTPServer(("0.0.0.0", port), make_handler(gs, page))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("UI: http://localhost:%d" % port)
    print("Data folder (settings + logs): %s" % DATA_DIR)
    if args.replay: gs.replay_file(args.replay, args.speed)
    elif cfg["connection"].get("auto_connect", True) and not args.no_connect: gs.connect()
    try:
        last = None
        while True:
            time.sleep(1); s = gs.snapshot()
            line = "%-42s lines=%d bad=%d cot=%d trackers=%s" % (s["conn_status"][:42], s["lines"], s["bad_sentences"], s["cot_sent"],
                                                                " ".join("%d(%.0fs)" % (t["id"], t["age_s"] or 0) for t in s["trackers"]))
            if line != last: print(line); last = line
    except KeyboardInterrupt: pass
    return 0

if __name__ == "__main__":
    sys.exit(main())
