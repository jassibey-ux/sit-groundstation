"""Full tracker IDs (17157 = 917 MHz slot 157): two bands on the same slot stay separate trackers,
and settings saved under the old short ID (157) migrate to the full ID."""
import csv, glob, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import Checks, CotListener, Sitgs, app_copy, base_config, multiband

check = Checks(); app = app_copy(); cot = CotListener()
cap = multiband(os.path.join(app, "capture.nmea"), os.path.join(app, "twoband.nmea"), (17157, 18157), limit=100)
base_config(app, cot={"destinations": [cot.dest()], "period_ms": 0}, cosmo={"enabled": False, "pairs": {"127.0.0.1:5253": "157"}},
            trackers={"157": {"callsign": "LEGACY-X"}})
gs = Sitgs(app, "--replay", cap, "--speed", "0")
try:
    st = gs.wait_status("replay finished"); cfg = gs.get("/api/config")["config"]
finally:
    gs.stop()
tr = {t["id"]: t for t in st["trackers"]}
check(sorted(tr) == [17157, 18157], "two trackers by full id: %s" % sorted(tr))
check((tr[17157]["carrier_mhz"], tr[17157]["id_slot"], tr[18157]["carrier_mhz"]) == (917, 157, 918), "carrier and slot decoded")
check(set(st["timeline"]) >= {"17157", "18157"}, "timeline rows keyed by full id")
files = [os.path.basename(p) for p in glob.glob(os.path.join(app, "logs", "csv", "*_ID*.csv"))]
check(any(f.endswith("_ID17157.csv") for f in files) and any(f.endswith("_ID18157.csv") for f in files), "separate split CSV per band: %s" % files)
row = list(csv.reader(open(glob.glob(os.path.join(app, "logs", "csv", "*_ID18157.csv"))[0])))[1]
check(row[12] == "18157.0", "Report_StationID is the full id")
uids = {e["uid"] for e in cot.events}
check({"TRK.17157", "TRK.18157"} <= uids and "TRK.157" not in uids, "CoT uids use the full id: %s" % sorted(uids))
check(cfg["trackers"] == {"17157": {"callsign": "LEGACY-X"}} and cfg["cosmo"]["pairs"] == {"127.0.0.1:5253": "17157"},
      "short-id callsign and pairing migrated to 17157")
check(any(e["uid"] == "TRK.17157" and e["callsign"] == "LEGACY-X" for e in cot.events), "migrated callsign used in CoT")
check.done()
