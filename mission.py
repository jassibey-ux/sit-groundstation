"""
mission.py - waypoint missions for sitgs: storage, DJI WPML/KMZ export, upload to a Cosmostreamer box

A mission is a dict:
  {"name": "survey1", "speed": 5, "height": 60, "finish": "goHome", "rth_height": 50, "drone_enum": 77, "drone_sub_enum": 0,
   "heading": "followWayline", "waypoints": [{"lat":.., "lon":.., "height": 60, "speed": 5}, ...]}

KMZ layout (DJI WPML 1.0.x, as used by DJI Fly/Pilot 2 waypoint missions):
  wpmz/template.kml   - editable template
  wpmz/waylines.wpml  - executable wayline
Upload: HTTP POST multipart to http://<box>/waypoints.php?action=upload  -> /boot/waypoints/<name>.kmz on the box
(or {"cmd":"wp","param":"file_upload","name":..,"value":<base64>} on the MQTT control topic).
"""
import base64, datetime as dt, io, json, math, os, re, time, urllib.request, uuid, zipfile

DRONE_ENUMS = {"Mavic 3 (77/0)": (77, 0), "Mavic 3 Cine (77/1)": (77, 1), "Mavic 3 Classic (77/2)": (77, 2), "Mavic 3 Pro (77/3)": (77, 3),
               "Mini 3 / 3 Pro (68/0)": (68, 0), "Mini 4 Pro (69/0)": (69, 0), "Air 3 (75/0)": (75, 0), "Matrice 30 (67/0)": (67, 0),
               "Matrice 300 RTK (60/0)": (60, 0), "Matrice 350 RTK (89/0)": (89, 0)}
FINISH_ACTIONS = ["goHome", "noAction", "autoLand", "gotoFirstWaypoint"]

def safe_name(n):
    n = re.sub(r"[^A-Za-z0-9_\-]+", "_", (n or "").strip())[:40]
    return n or "mission"

def haversine(a, b):
    R = 6371000.0; p1, p2 = math.radians(a[0]), math.radians(b[0]); dp = p2 - p1; dl = math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))

def mission_stats(m):
    wps = m.get("waypoints", []); dist = 0.0
    for i in range(1, len(wps)): dist += haversine((wps[i - 1]["lat"], wps[i - 1]["lon"]), (wps[i]["lat"], wps[i]["lon"]))
    spd = max(0.5, float(m.get("speed", 5)))
    return {"distance_m": round(dist, 1), "duration_s": round(dist / spd, 1), "count": len(wps)}

def _mission_config(m):
    return ("<wpml:missionConfig><wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode><wpml:finishAction>%s</wpml:finishAction>"
            "<wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost><wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>"
            "<wpml:takeOffSecurityHeight>%.1f</wpml:takeOffSecurityHeight><wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>"
            "<wpml:globalRTHHeight>%.1f</wpml:globalRTHHeight><wpml:droneInfo><wpml:droneEnumValue>%d</wpml:droneEnumValue>"
            "<wpml:droneSubEnumValue>%d</wpml:droneSubEnumValue></wpml:droneInfo></wpml:missionConfig>"
            ) % (m.get("finish", "goHome"), float(m.get("takeoff_height", 20)), float(m.get("speed", 5)), float(m.get("rth_height", 50)),
                 int(m.get("drone_enum", 77)), int(m.get("drone_sub_enum", 0)))

def build_kmz(m):
    now = int(time.time() * 1000); wps = m.get("waypoints", []); speed = float(m.get("speed", 5)); gh = float(m.get("height", 60))
    heading = m.get("heading", "followWayline")
    head = ('<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2"><Document>'
            '<wpml:author>sitgs</wpml:author><wpml:createTime>%d</wpml:createTime><wpml:updateTime>%d</wpml:updateTime>' % (now, now))
    tpl = [head, _mission_config(m),
           '<Folder><wpml:templateType>waypoint</wpml:templateType><wpml:templateId>0</wpml:templateId>'
           '<wpml:waylineCoordinateSysParam><wpml:coordinateMode>WGS84</wpml:coordinateMode><wpml:heightMode>relativeToStartPoint</wpml:heightMode></wpml:waylineCoordinateSysParam>'
           '<wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed><wpml:globalHeight>%.1f</wpml:globalHeight><wpml:caliFlightEnable>0</wpml:caliFlightEnable>'
           '<wpml:gimbalPitchMode>manual</wpml:gimbalPitchMode><wpml:globalWaypointHeadingParam><wpml:waypointHeadingMode>%s</wpml:waypointHeadingMode>'
           '<wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode></wpml:globalWaypointHeadingParam>'
           '<wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode><wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>' % (speed, gh, heading)]
    wl = [head, _mission_config(m),
          '<Folder><wpml:templateId>0</wpml:templateId><wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode><wpml:waylineId>0</wpml:waylineId>'
          '<wpml:distance>%.1f</wpml:distance><wpml:duration>%.1f</wpml:duration><wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>' % (
              mission_stats(m)["distance_m"], mission_stats(m)["duration_s"], speed)]
    for i, w in enumerate(wps):
        h = float(w.get("height", gh)); s = float(w.get("speed", speed))
        pt = '<Point><coordinates>%.8f,%.8f</coordinates></Point>' % (float(w["lon"]), float(w["lat"]))
        tpl.append('<Placemark>%s<wpml:index>%d</wpml:index><wpml:ellipsoidHeight>%.1f</wpml:ellipsoidHeight><wpml:height>%.1f</wpml:height>'
                   '<wpml:useGlobalHeight>%d</wpml:useGlobalHeight><wpml:useGlobalSpeed>%d</wpml:useGlobalSpeed><wpml:waypointSpeed>%.1f</wpml:waypointSpeed>'
                   '<wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam><wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam><wpml:useStraightLine>1</wpml:useStraightLine></Placemark>'
                   % (pt, i, h, h, 1 if h == gh else 0, 1 if s == speed else 0, s))
        wl.append('<Placemark>%s<wpml:index>%d</wpml:index><wpml:executeHeight>%.1f</wpml:executeHeight><wpml:waypointSpeed>%.1f</wpml:waypointSpeed>'
                  '<wpml:waypointHeadingParam><wpml:waypointHeadingMode>%s</wpml:waypointHeadingMode><wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode></wpml:waypointHeadingParam>'
                  '<wpml:waypointTurnParam><wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode><wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist></wpml:waypointTurnParam>'
                  '<wpml:useStraightLine>1</wpml:useStraightLine></Placemark>' % (pt, i, h, s, heading))
    tpl.append('</Folder></Document></kml>'); wl.append('</Folder></Document></kml>')
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("wpmz/template.kml", "".join(tpl)); z.writestr("wpmz/waylines.wpml", "".join(wl))
    return buf.getvalue()

class MissionStore:
    def __init__(self, folder):
        self.folder = folder; os.makedirs(folder, exist_ok=True)
    def path(self, name): return os.path.join(self.folder, safe_name(name) + ".json")
    def list(self):
        out = []
        for f in sorted(os.listdir(self.folder)):
            if f.endswith(".json"):
                try:
                    m = json.load(open(os.path.join(self.folder, f))); m["stats"] = mission_stats(m); out.append(m)
                except (ValueError, OSError): pass
        return out
    def get(self, name):
        p = self.path(name)
        return json.load(open(p)) if os.path.exists(p) else None
    def save(self, m):
        m["name"] = safe_name(m.get("name")); m["updated"] = dt.datetime.now().isoformat(timespec="seconds")
        with open(self.path(m["name"]), "w") as f: json.dump(m, f, indent=2)
        with open(os.path.join(self.folder, m["name"] + ".kmz"), "wb") as f: f.write(build_kmz(m))
        return m
    def delete(self, name):
        for ext in (".json", ".kmz"):
            p = os.path.join(self.folder, safe_name(name) + ext)
            if os.path.exists(p): os.remove(p)

def upload_to_box(host, name, kmz_bytes, timeout=15):
    """multipart POST to the box's waypoints.php (same call its own web UI makes)"""
    boundary = "----sitgs" + uuid.uuid4().hex
    body = (("--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"%s.kmz\"\r\nContent-Type: application/vnd.google-earth.kmz\r\n\r\n" % (boundary, name)).encode()
            + kmz_bytes + ("\r\n--%s--\r\n" % boundary).encode())
    req = urllib.request.Request("http://%s/waypoints.php?action=upload" % host, data=body,
                                 headers={"Content-Type": "multipart/form-data; boundary=" + boundary})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read().decode("utf8", "replace").strip()

def mqtt_upload_payload(name, kmz_bytes):
    return json.dumps({"cmd": "wp", "param": "file_upload", "name": name + ".kmz", "value": base64.b64encode(kmz_bytes).decode()})

def mediamtx_yaml(boxes, record=True, record_path="./recordings/%path/%Y-%m-%d_%H-%M-%S-%f", extra_paths=None):
    """Generate a mediamtx.yml that pulls every box's RTSP stream and (optionally) records it."""
    lines = ["# generated by sitgs %s" % dt.datetime.now().isoformat(timespec="seconds"),
             "rtsp: yes", "rtspAddress: :8554", "webrtc: yes", "webrtcAddress: :8889", "hls: yes", "hlsAddress: :8888", "api: yes", "apiAddress: :9997",
             "record: %s" % ("yes" if record else "no"), "recordPath: %s" % record_path, "recordFormat: fmp4", "recordSegmentDuration: 10m", "", "paths:"]
    for b in boxes:
        if b.get("transport") != "udp": continue
        pname = safe_name(b.get("name") or b["host"].replace(".", "-")).lower()
        lines += ["  %s:" % pname, "    source: rtsp://%s:554/video" % b["host"], "    sourceOnDemand: no", "    sourceProtocol: tcp"]
    for p in (extra_paths or []): lines += ["  %s:" % safe_name(p["name"]).lower(), "    source: %s" % p["url"]]
    return "\n".join(lines) + "\n"
