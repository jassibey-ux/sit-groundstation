"""Serial watchdog: a read that never returns (a wedged Windows USB driver) is cancelled and the port
reopened; a thread that ignores the cancel is abandoned and replaced."""
import json, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import Checks, app_copy

check = Checks(); app = app_copy(); sys.path.insert(0, app)
import sitgs

class FakeSer:
    instances = []
    def __init__(self, honours_cancel):
        self.honours = honours_cancel; self.release = threading.Event(); self.reads = 0; self.cancelled = False
        FakeSer.instances.append(self)
    def __enter__(self): return self
    def __exit__(self, *a): return False
    in_waiting = 0
    def read(self, n):
        self.reads += 1
        if self.reads < 6: time.sleep(0.1); return b""
        self.release.wait()
        return b""          # like serialwin32: a cancelled read returns empty instead of raising
    def cancel_read(self):
        self.cancelled = True
        if self.honours: self.release.set()

def run(honours):
    FakeSer.instances.clear()
    sitgs.serial.serial_for_url = lambda port, **kw: FakeSer(honours)
    cfg = json.loads(json.dumps(sitgs.DEFAULT_CONFIG))
    cfg["connection"].update(type="serial", port="FAKE0"); cfg["cosmo"]["enabled"] = False
    cfg["log"]["nmea_enabled"] = False; cfg["log"]["split_enabled"] = False
    gs = sitgs.GroundStation(cfg, os.path.join(app, "wd.json")); gs.connect()
    t0 = time.time(); ok = False
    while time.time() - t0 < 30:
        time.sleep(0.5)
        log = list(gs.debug)
        recovered = len(FakeSer.instances) >= 2 and gs.conn_status == "connected FAKE0"
        if recovered and (honours or any("abandoning" in l for l in log)): ok = True; break
    for s in FakeSer.instances: s.release.set()
    check(ok, "stuck read recovered (read honours cancel=%s) in %.1f s, port opened %d times"
          % (honours, time.time() - t0, len(FakeSer.instances)))
    gs.disconnect(); gs.cosmo.stop()

run(True); run(False)
check.done()
