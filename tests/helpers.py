"""Shared test plumbing. Each test runs sitgs from a throwaway copy of the app, so settings and
logs never touch the repo; sitgs is started with --no-browser."""
import json, os, re, shutil, socket, subprocess, sys, tempfile, threading, time, urllib.error, urllib.request

ROOT = os.environ.get("SITGS_DIR") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP_FILES = ["sitgs.py", "cosmo.py", "mission.py", "apppaths.py", "sitgs_ui.html", "capture.nmea", "cosmo_sim.py"]
APP_DIRS = ["serial", "static"]

class Checks:
    def __init__(self): self.fails = []
    def __call__(self, cond, msg):
        print(("PASS " if cond else "FAIL ") + msg, flush=True)
        if not cond: self.fails.append(msg)
    def done(self):
        print("RESULT:", "ALL PASS" if not self.fails else "%d FAIL: %s" % (len(self.fails), self.fails))
        sys.exit(1 if self.fails else 0)

def app_copy():
    d = tempfile.mkdtemp(prefix="sitgs-test-")
    for f in APP_FILES: shutil.copy2(os.path.join(ROOT, f), d)
    for sub in APP_DIRS: shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub), ignore=shutil.ignore_patterns("__pycache__"))
    return d

def blocks(path):
    """Report blocks of an NMEA capture: lists of lines from $RFMSGFROM to $RFMSGEND."""
    out, cur = [], []
    for line in open(path):
        line = line.rstrip("\n")
        if line.startswith("$RFMSGFROM"):
            if cur: out.append(cur)
            cur = []
        if line: cur.append(line)
    if cur: out.append(cur)
    return out

def multiband(src, dst, ids, limit=200):
    """Interleave the 17157 blocks of src re-addressed to each id in ids (header checksum is empty)."""
    with open(dst, "w") as f:
        for b in blocks(src)[:limit]:
            for tid in ids:
                f.write("\n".join(l.replace("$RFMSGFROM,17157", "$RFMSGFROM,%d" % tid) for l in b) + "\n")
    return dst

def base_config(app, **over):
    cfg = {"connection": {"auto_connect": False}, "web": {"open_browser": False},
           "cosmo": {"enabled": False}, "cot": {"destinations": []}, "trackers": {}}
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(cfg.get(k), dict): cfg[k].update(v)
        else: cfg[k] = v
    path = os.path.join(app, "sitgs.json"); json.dump(cfg, open(path, "w"), indent=2)
    return path

def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p

class CotListener:
    """Collects CoT events: (uid, type, callsign, remarks, altsrc, hae)."""
    def __init__(self):
        self.port = free_port(); self.events = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.sock.bind(("127.0.0.1", self.port)); self.sock.settimeout(0.3)
        threading.Thread(target=self._run, daemon=True).start()
    def _run(self):
        while True:
            try: d, _ = self.sock.recvfrom(65535)
            except socket.timeout: continue
            except OSError: return
            x = d.decode()
            g = lambda pat: (re.search(pat, x) or [None, None])[1]
            self.events.append({"uid": g(r'uid="([^"]+)"'), "type": g(r'type="([^"]+)"'), "callsign": g(r'callsign="([^"]+)"'),
                                "remarks": g(r"<remarks>(.*?)</remarks>"), "altsrc": g(r'altsrc="([^"]+)"'), "hae": float(g(r'hae="([^"]+)"')),
                                "xml": x})
    def dest(self): return {"host": "127.0.0.1", "port": self.port, "iface": "0.0.0.0", "ttl": 2}

class Sitgs:
    def __init__(self, app, *args):
        self.app = app; self.port = free_port()
        self.proc = subprocess.Popen([sys.executable, os.path.join(app, "sitgs.py"), "--config", os.path.join(app, "sitgs.json"),
                                      "--web-port", str(self.port), "--no-browser"] + list(args),
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        for _ in range(40):
            try: self.get("/api/config"); break
            except Exception: time.sleep(0.25)
    def get(self, path):
        return json.load(urllib.request.urlopen("http://localhost:%d%s" % (self.port, path), timeout=5))
    def raw(self, path):
        return urllib.request.urlopen("http://localhost:%d%s" % (self.port, path), timeout=5).read()
    def post(self, path, body):
        r = urllib.request.Request("http://localhost:%d%s" % (self.port, path), data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
        try: return json.load(urllib.request.urlopen(r, timeout=10))
        except urllib.error.HTTPError as e: return json.load(e)
    def wait_status(self, status, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.get("/api/state?trails=0")
            if s["conn_status"] == status: return s
            time.sleep(0.3)
        raise TimeoutError("status %r not reached" % status)
    def stop(self):
        self.proc.terminate()
        try: self.proc.wait(5)
        except subprocess.TimeoutExpired: self.proc.kill()

def nmea_ok(line):
    if "*" not in line: return False
    body, cks = line[1:].split("*", 1)
    x = 0
    for ch in body: x ^= ord(ch)
    return cks.strip().upper() == "%02X" % x
