"""Live-serial soak / many-tracker run (manual, not in CI). Feeds a pty at real rate (1 report/s per
tracker) into sitgs - a frozen build (--exe) or source (default) - and samples /api/state.

    python tests/soak.py --trackers 40 --duration 2700            # 45 min, source
    python tests/soak.py --exe release/sitgs --trackers 2 --stop 3 --stop-after 60
Options: --stop N stops the last N trackers after --stop-after seconds (they should go amber, then red).
Leaves the instance up on --port for a browser until the run ends. macOS/Linux only (pty).
"""
import argparse, json, os, subprocess, sys, threading, time, tty, urllib.request
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import app_copy, base_config
from make_multi import second, tracker_ids

ap = argparse.ArgumentParser()
ap.add_argument("--exe"); ap.add_argument("--trackers", type=int, default=40); ap.add_argument("--duration", type=int, default=300)
ap.add_argument("--stop", type=int, default=0); ap.add_argument("--stop-after", type=int, default=60); ap.add_argument("--port", type=int, default=8170)
a = ap.parse_args()

app = app_copy(); base_config(app, connection={"type": "serial", "baud": 115200, "auto_connect": True})
master, slave = os.openpty(); tty.setraw(slave); dev = os.ttyname(slave)
ids = tracker_ids(a.trackers); stopped = set(); t_start = time.time()
def feed():
    while True:
        t = time.time()
        live = [i for i in ids if i not in stopped]
        os.write(master, ("\n".join(second(live, t)) + "\n").encode())
        time.sleep(max(0, 1 - (time.time() - t)))
threading.Thread(target=feed, daemon=True).start()
cmd = [a.exe] if a.exe else [sys.executable, os.path.join(app, "sitgs.py")]
proc = subprocess.Popen(cmd + ["--config", os.path.join(app, "sitgs.json"), "--port", dev, "--web-port", str(a.port), "--no-browser"],
                        cwd=app, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
print("sitgs on http://localhost:%d  serial %s  %d trackers  data %s" % (a.port, dev, a.trackers, app), flush=True)

def state(q="?trails=0"):
    return urllib.request.urlopen("http://localhost:%d/api/state%s" % (a.port, q), timeout=5).read()
fails = []; prev = None
try:
    time.sleep(5)
    while time.time() - t_start < a.duration:
        el = time.time() - t_start
        if a.stop and not stopped and el > a.stop_after:
            stopped.update(ids[-a.stop:]); print("stopped trackers %s at %.0f s" % (sorted(stopped), el), flush=True)
        raw = state(); s = json.loads(raw)
        live = [t for t in s["trackers"] if (t["age_s"] or 99) < 3]
        print("t=%4.0fs trackers=%d live(<3s)=%d reports=%d lines=%d state=%d B (with trails %d B) conn=%s"
              % (el, len(s["trackers"]), len(live), s["reports"], s["lines"], len(raw), len(state("")), s["conn_status"]), flush=True)
        if prev and s["reports"] <= prev: fails.append("reports stalled at %.0f s" % el)
        prev = s["reports"]; time.sleep(15)
    s = json.loads(state())
    ages = {t["id"]: t["age_s"] for t in s["trackers"]}
    if stopped: print("ages of stopped trackers:", {i: ages.get(i) for i in sorted(stopped)})
    data_dir = os.path.dirname(os.path.abspath(a.exe)) if a.exe else app   # a frozen build writes next to itself
    ev = open(os.path.join(data_dir, "logs", "sitgs-events.log")).read()
    print("watchdog trips: %d" % ev.count("WATCHDOG"))
finally:
    proc.terminate()
print("RESULT:", "PASS" if not fails else "FAIL %s" % fails)
sys.exit(1 if fails else 0)
