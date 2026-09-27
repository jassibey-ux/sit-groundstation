"""Per-tracker tactical symbols on every CoT feed, drone symbols (paired / unpaired), target in remarks,
symbol validation, associations sidecar, and survival of a short Cosmostreamer discovery answer."""
import csv, glob, html, os, re, shutil, socket, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import Checks, CotListener, Sitgs, app_copy, base_config, free_port, multiband

check = Checks(); app = app_copy(); feeds = [CotListener(), CotListener()]
cap = multiband(os.path.join(app, "capture.nmea"), os.path.join(app, "twoband.nmea"), (17157, 18157), limit=200)
pa, pb = free_port(), free_port()
base_config(app, cot={"destinations": [f.dest() for f in feeds], "type": "a-f-A-C-H-q"},
            trackers={"17157": {"target": "Mavic 3 #2 <red> & co", "callsign": "RED-2", "cot_type": "a-h-A-M-F-Q"}, "19005": {}},
            cosmo={"enabled": True, "discovery": False, "manual_hosts": ["127.0.0.1:%d" % pa, "127.0.0.1:%d" % pb],
                   "pairs": {"127.0.0.1:%d" % pa: "17157"}, "box_types": {"127.0.0.1:%d" % pb: "a-u-A-M-H-Q"}, "cot_unpaired": True})
sims = [subprocess.Popen([sys.executable, os.path.join(app, "cosmo_sim.py"), "--port", str(p), "--name", n, "--lat", "38.8447", "--lon", "-77.0764"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for p, n in ((pa, "Alpha"), (pb, "Bravo"))]
gs = Sitgs(app, "--replay", cap, "--speed", "10")
try:
    for _ in range(40):
        time.sleep(1)
        if len(feeds[0].events) > 40 and any("src=DJI" in (e["remarks"] or "") for e in feeds[0].events): break
    time.sleep(3)
    pairs = lambda f, src: {(e["uid"], e["type"]) for e in f.events if src in (e["remarks"] or "")}
    check(("TRK.17157", "a-h-A-M-F-Q") in pairs(feeds[0], "src=LoRa"), "17157 sent with its own symbol")
    check(("TRK.18157", "a-f-A-C-H-q") in pairs(feeds[0], "src=LoRa"), "18157 falls back to the default symbol")
    check(pairs(feeds[0], "src=LoRa") == pairs(feeds[1], "src=LoRa"), "both output feeds carry the same symbols")
    r17 = next(html.unescape(e["remarks"]) for e in feeds[0].events if e["uid"] == "TRK.17157" and "src=LoRa" in e["remarks"])
    check(r17.startswith("target=Mavic 3 #2 <red> & co id=17157"), "target leads CoT remarks (XML-escaped on the wire)")
    try:
        import xml.etree.ElementTree as ET
        for e in feeds[0].events[:50]: ET.fromstring(e["xml"])
        check(True, "CoT XML is well-formed with odd target text")
    except ImportError:
        print("SKIP XML parse check (no pyexpat)")
    check(("TRK.17157", "a-h-A-M-F-Q") in pairs(feeds[0], "src=DJI"), "paired drone uses its tracker's symbol")
    check(any(u.startswith("DJI.Bravo") and t == "a-u-A-M-H-Q" for u, t in pairs(feeds[0], "src=DJI")), "unpaired drone uses its own symbol")

    before = gs.get("/api/config")["config"]
    check(gs.post("/api/config", {"trackers": {"17157": {"cot_type": "hostile drone"}}}).get("ok") is False, "malformed symbol rejected")
    check(gs.post("/api/config", {"cot": {"type": ""}}).get("ok") is False and gs.post("/api/config", {"trackers": {"abc": {}}}).get("ok") is False,
          "empty default symbol and non-numeric tracker id rejected")
    check(gs.get("/api/config")["config"] == before, "config unchanged after rejected patches")
    c = gs.get("/api/config")
    check(len(c["symbol_types"]) == 15 and len(c["symbol_affiliations"]) == 7, "picker catalogue served")

    if shutil.which("lsof"):
        out = subprocess.run(["lsof", "-nP", "-iUDP", "-a", "-p", str(gs.proc.pid)], capture_output=True, text=True).stdout
        k = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for p in {int(m) for m in re.findall(r":(\d+)\s*$", out, re.M)}: k.sendto(bytes([0x52, 0]) + b"\0" * 10 + bytes([1]), ("127.0.0.1", p))
        time.sleep(3); st = gs.get("/api/state?trails=0"); n = st["cosmo"]["packets"]; time.sleep(3)
        check(any("cosmo bad packet" in d for d in st["debug"]) and gs.get("/api/state?trails=0")["cosmo"]["packets"] > n,
              "short discovery answer logged, drone listener keeps receiving")
    else:
        print("SKIP bad-packet check (no lsof)")
finally:
    gs.stop(); [s.kill() for s in sims]

assoc = glob.glob(os.path.join(app, "logs", "csv", "*_associations.csv"))
check(len(assoc) == 1, "associations sidecar written")
ids = {r["Tracker_ID"]: r for r in csv.DictReader(open(assoc[0]))}
check(set(ids) == {"17157", "18157", "19005"} and ids["19005"]["Heard_This_Session"] == "no", "sidecar lists heard + pre-registered trackers")
check.done()
