"""Cosmostreamer client: bring-up probe, firmware-style value clearing, packet capture and replay,
a box with no drone, unknown altitude for unpaired drones, the mission-command guard, robustness.
Every capture of a real box placed in tests/data/cosmo_*.jsonl is replayed through sitgs too."""
import glob, json, os, socket, struct, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import Checks, CotListener, Sitgs, app_copy, base_config, free_port

check = Checks(); app = app_copy(); sys.path.insert(0, app)
import cosmo
HERE = os.path.dirname(os.path.abspath(__file__))

def sim(*args):
    return subprocess.Popen([sys.executable, os.path.join(app, "cosmo_sim.py")] + list(args), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def wait(pred, timeout=15, step=0.3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = pred()
        if r: return r
        time.sleep(step)
    return None

def boxes(gs): return gs.get("/api/state?trails=0")["cosmo"]["boxes"]

# --- unit-level: firmware app types, short binary packets
check(cosmo.APP_TYPES[32] == "DJI Fly drone (RC-N) Pro" and 32 in cosmo.TELEMETRY_TYPES, "PRO licence app type 32 known")
b = cosmo.Box("x", 1); cosmo.CosmoClient.parse_binary(b, bytes([6, 0]) + b"\x00" * 10)
check("pan" in b.bin and "drone_yaw" not in b.bin, "short binary packet parsed as far as it goes, no exception")

# --- probe against a simulated box
port = free_port(); s1 = sim("--port", str(port), "--name", "Probe1")
try:
    cap = os.path.join(app, "probe.jsonl"); lines = []
    rep = cosmo.probe(["127.0.0.1:%d" % port], seconds=4, capture=cap, out=lines.append)
finally:
    s1.kill()
bx = [x for x in rep["boxes"] if x.get("answered")]
check(len(bx) == 1 and bx[0]["name"] == "Probe1" and bx[0]["app_type"] == 22 and bx[0]["version"] == "0.24.11", "probe decodes the discovery answer")
check("gps_lat" in bx[0]["keys"] and any(k.startswith("ttype 0, 56 bytes") for k in bx[0]["binary"]), "probe lists text keys and binary layout")
check("rtsp" in bx[0] and "waypoints" in bx[0] and "http" in bx[0], "probe checks web UI, waypoint upload and RTSP")
rows = [json.loads(l) for l in open(cap)]
check({r["dir"] for r in rows} == {"rx", "tx"} and len(rows) > 10, "probe capture has both directions (%d datagrams)" % len(rows))
check(any("text keys" in l for l in lines), "probe prints a readable report")

# --- sitgs against a box with a link drop; capture on
cot = CotListener(); pa, pb, pc = free_port(), free_port(), free_port()
base_config(app, cot={"destinations": [cot.dest()], "period_ms": 200},
            cosmo={"enabled": True, "discovery": False, "cot_unpaired": True,
                   "manual_hosts": ["127.0.0.1:%d" % pa, "127.0.0.1:%d" % pb, "127.0.0.1:%d" % pc]})
sims = [sim("--port", str(pa), "--name", "Alpha", "--link-drop", "6:6"), sim("--port", str(pb), "--name", "Empty", "--no-drone")]
gs = Sitgs(app, "--cosmo-capture")
try:
    check(wait(lambda: len([x for x in boxes(gs) if x["online"]]) == 2), "both boxes discovered via manual host")
    al = lambda: next(x for x in boxes(gs) if x["name"] == "Alpha")
    check(wait(lambda: al()["connected"] and al()["lat"] is not None), "Alpha telemetry live")
    check(al()["found_via"] == "manual host", "box records how it was found")
    check(wait(lambda: al()["camera_detected"] == "0" and al()["lat"] is None and not al()["fix"], timeout=12),
          "link drop: stale position cleared (firmware rule), no fix")
    check(wait(lambda: al()["lat"] is not None, timeout=12), "link back: telemetry returns")
    em = next(x for x in boxes(gs) if x["name"] == "Empty")
    check(em["online"] and not em["connected"] and em["lat"] is None, "box without a drone: online, not connected, no position")
    time.sleep(1)
    check(not [e for e in cot.events if "Empty" in (e["uid"] or "")], "no CoT for a box without a drone")
    alpha = [e for e in cot.events if e["uid"] == "DJI.Alpha"]
    check(alpha and all(e["hae"] == 9999999.0 and e["altsrc"] == "???" and "alt=unknown" in e["remarks"] for e in alpha),
          "unpaired drone CoT: altitude unknown (no take-off MSL), not a wrong value")
    d = gs.get("/api/cosmo/diag")
    da = next(x for x in d["boxes"] if x["key"].endswith(":%d" % pa))
    check(da["raw_text"] and "gps_lat" in da["keys"] and da["counts"].get("text telemetry", 0) > 3, "diagnostics expose raw text and all keys")
    check(d["capture"]["on"] and d["capture"]["datagrams"] > 20, "capture running (%d datagrams)" % d["capture"]["datagrams"])

    r = gs.post("/api/mission/cmd", {"box_key": "127.0.0.1:%d" % pa, "action": "start", "name": "m1"})
    check(r.get("ok") is False and "guesses" in r.get("error", ""), "unconfirmed mission start refused")
    r = gs.post("/api/mission/cmd", {"box_key": "127.0.0.1:%d" % pa, "action": "rth"})
    check(r.get("ok") is True and "rth" in r.get("sent", ""), "RTH (box web UI string) still sendable")

    check(gs.post("/api/cosmo/capture", {"on": False}).get("on") is False, "capture can be switched off at runtime")
    capfile = wait(lambda: gs.get("/api/cosmo/diag")["capture"]["on"] is False and
                   sorted(glob.glob(os.path.join(app, "logs", "cosmo", "*_cosmo.jsonl"))), timeout=5)
    dl = gs.raw("/api/cosmo/capture")
    check(capfile and dl and len(dl.splitlines()) > 20, "capture file written and downloadable")
finally:
    gs.stop(); [x.kill() for x in sims]

# --- replay that capture as if it were a real box, through a fresh sitgs
def replay_check(path, label, need_keys=False):
    rows = [json.loads(l) for l in open(path)]
    rx = [r for r in rows if r["dir"] == "rx" and len(bytes.fromhex(r["hex"])) > 12]
    answered = {r["peer"] for r in rx if bytes.fromhex(r["hex"])[12] == 1}
    # replay the box that sent the most telemetry
    peer = max(answered, key=lambda pr: sum(1 for r in rx if r["peer"] == pr and bytes.fromhex(r["hex"])[12] in (6, 7)))
    port = free_port(); app2 = app_copy()
    base_config(app2, cosmo={"enabled": True, "discovery": False, "manual_hosts": ["127.0.0.1:%d" % port]})
    sp = sim("--port", str(port), "--replay", path, "--replay-peer", peer)
    g = Sitgs(app2)
    try:
        bx = wait(lambda: [x for x in boxes(g) if x["online"]], timeout=15)
        mine = [r for r in rx if r["peer"] == peer]
        ans = next(bytes.fromhex(r["hex"])[12:] for r in mine if bytes.fromhex(r["hex"])[12] == 1)
        check(bx and bx[0]["app_type"] == ans[1], "%s: replayed box comes online with its app type %s" % (label, ans[1]))
        time.sleep(min(60, mine[-1]["t"] - mine[0]["t"]) + 2)   # let the whole capture play once
        d = g.get("/api/cosmo/diag")
        keys = set()
        for r in mine:
            p = bytes.fromhex(r["hex"])[12:]
            if p[:1] == b"\x07":
                keys |= {kv.split("=")[0].strip() for kv in p[1:].decode("ascii", "replace").split(",") if "=" in kv}
        seen = set(d["boxes"][0]["keys"]) if d["boxes"] else set()
        check(keys <= seen and (keys or not need_keys), "%s: every text key in the capture reaches sitgs (%d keys)" % (label, len(keys)))
        check(d["errors"] == 0, "%s: no parse errors" % label)
    finally:
        g.stop(); sp.kill()

replay_check(capfile[-1], "own capture", need_keys=True)
for real in sorted(glob.glob(os.path.join(HERE, "data", "cosmo_*.jsonl"))):
    replay_check(real, os.path.basename(real))

# --- datagrams from unknown hosts / with bad headers are counted, not silently lost
app3 = app_copy(); port = free_port()
base_config(app3, cosmo={"enabled": True, "discovery": False, "manual_hosts": ["127.0.0.1:%d" % port]})
g = Sitgs(app3)
try:
    time.sleep(1.5)
    out = subprocess.run(["lsof", "-nP", "-iUDP", "-a", "-p", str(g.proc.pid)], capture_output=True, text=True).stdout if sys.platform != "win32" else ""
    import re
    ports = {int(m) for m in re.findall(r":(\d+)\s*$", out, re.M)}
    k = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for p in ports:
        k.sendto(b"\x00\x01garbage", ("127.0.0.1", p))
        k.sendto(cosmo.frame(bytes([7]) + b"gps_lat=1"), ("127.0.0.1", p))
    if ports:
        d = wait(lambda: (lambda d: d if d["drops"].get("bad header") and d["drops"].get("unknown host") else None)(g.get("/api/cosmo/diag")), timeout=5)
        check(bool(d) and d["unknown"], "bad-header and unknown-host datagrams counted and shown")
    else:
        print("SKIP drop counters (no lsof)")
finally:
    g.stop()
check.done()
