"""Association-list processing applied live: record on/off, per-tracker altitude source, per-target raw
and processed NMEA files, unlisted-tracker policy. The combined NMEA log and 5.2 CSV header stay untouched."""
import csv, glob, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import Checks, CotListener, Sitgs, app_copy, base_config, blocks, multiband, nmea_ok

check = Checks(); app = app_copy(); cot = CotListener()
cap = multiband(os.path.join(app, "capture.nmea"), os.path.join(app, "three.nmea"), (17157, 18157, 19157), limit=60)
base_config(app,
            cot={"destinations": [cot.dest()], "period_ms": 0, "geoid_auto_update": False, "geoid_height_m": -33.0},
            barometer={"mode": "as-is", "auto_update": "disable", "ref_pressure_bar": 1.02, "ref_temp_c": 20.0, "ref_altitude_m": 0.0},
            unlisted_trackers="ignore",
            trackers={"17157": {"processing": {"record": False, "alt_source": "gps", "nmea_file": True}},
                      "18157": {"target": "Blue Mavic #1", "processing": {"alt_source": "recompute", "nmea_file": True, "nmea_processed": True}}})
gs = Sitgs(app, "--replay", cap, "--speed", "0")
try:
    st = gs.wait_status("replay finished"); time.sleep(1.5)
    ids = sorted(t["id"] for t in st["trackers"])
    check(ids == [17157, 18157], "unlisted 19157 ignored, listed trackers live: %s" % ids)
    check(st["ignored"].get("19157", 0) == 60, "ignored IDs counted: %s" % st["ignored"])
    t17 = next(t for t in st["trackers"] if t["id"] == 17157); t18 = next(t for t in st["trackers"] if t["id"] == 18157)
    check(t17["record"] is False and t17["csv_rows"] == 0 and t17["last_row"] is not None, "17157 record off: shown, 0 CSV rows, latest row still visible")
    check(t18["record"] is True and t18["csv_rows"] == 60 and t18["alt_mode"] == "recompute", "18157 recorded 60 rows with recompute altitude")
    small = len(gs.raw("/api/state?trails=0")); full = len(gs.raw("/api/state"))
    check(small < full and gs.get("/api/state?trails=0")["trails"] == {}, "trails=0 drops trails (%d vs %d bytes)" % (small, full))

    # validation
    r = gs.post("/api/config", {"trackers": {"18157": {"processing": {"alt_source": "moon"}}}})
    check(r.get("ok") is False and "altitude source" in r.get("error", ""), "bad altitude source rejected")
    r = gs.post("/api/config", {"trackers": {"18157": {"processing": {"record": "yes"}}}})
    check(r.get("ok") is False, "non-boolean switch rejected")
    check(gs.post("/api/config", {"unlisted_trackers": "maybe"}).get("ok") is False, "bad unlisted policy rejected")

    # add the ignored tracker to the list, replay again: it becomes live
    trk = gs.get("/api/config")["config"]["trackers"]; trk["19157"] = {"target": "Late add"}
    gs.post("/api/config", {"trackers": trk}); gs.post("/api/replay", {"path": cap, "speed": 0})
    time.sleep(1); st2 = gs.wait_status("replay finished")
    check(19157 in [t["id"] for t in st2["trackers"]] and "19157" not in st2["ignored"], "after adding 19157 to the list it is live")
finally:
    gs.stop()

csvdir = os.path.join(app, "logs", "csv"); nmeadir = os.path.join(app, "logs", "nmea")
check(not glob.glob(os.path.join(csvdir, "*_ID17157.csv")), "no split CSV for 17157 (record off)")
check(not [f for f in os.listdir(nmeadir) if "_ID17157" in f], "no per-target NMEA for 17157 although nmea_file is on (record off)")
f18 = glob.glob(os.path.join(csvdir, "*_ID18157.csv"))[0]; rows = list(csv.reader(open(f18)))
check(rows[0] == ["Measurement_DateTime", "Measurement_ReceivedDateTime", "GPS_FixValid", "GPS_lat", "GPS_lon", "GPS_alt", "GPS_HDOP",
                  "GPS_SOG", "GPS_COG", "Barometer_Pressure", "Barometer_Temperature", "Barometer_Altitude", "Report_StationID", "RF_RSSI"],
      "split CSV header unchanged (manual 5.2)")

sys.path.insert(0, app); import sitgs
slp = sitgs.sea_level_pressure(1.02e5, 20.0, 0.0)
# expected from the full-precision $BAROALT pressure (the CSV column is rounded to 1e-5 bar)
p18 = [float(l.split(",")[1]) for b in blocks(cap) if b[0].startswith("$RFMSGFROM,18157") for l in b if l.startswith("$BAROALT")]
exp = ["%.1f" % sitgs.baro_altitude(p, 20.0, slp) for p in p18[:60]]
check([r[11] for r in rows[1:61]] == exp, "CSV Barometer_Altitude = recomputed altitude for 18157 (e.g. %s)" % exp[0])

combined = glob.glob(os.path.join(nmeadir, "*_RFReceiverRSSILog.nmea"))[0]
comb_blocks = blocks(combined)
raw18 = [f for f in os.listdir(nmeadir) if f.endswith("_ID18157_Blue_Mavic_1.nmea")]
check(len(raw18) == 1, "per-target raw NMEA named by target: %s" % raw18)
mine = [b for b in comb_blocks if b[0].startswith("$RFMSGFROM,18157")]
check(blocks(os.path.join(nmeadir, raw18[0])) == mine[:len(blocks(os.path.join(nmeadir, raw18[0])))] and len(mine) == 120,
      "per-target raw NMEA blocks are identical to 18157's blocks in the combined log (incl. $RXTIMESTAMP)")
# combined log = everything received (all 3 trackers, twice) + injected $RXTIMESTAMP lines
received = [l.rstrip("\n") for l in open(cap) if l.strip()] * 2
logged = [l.rstrip("\n") for l in open(combined) if not l.startswith("$RXTIMESTAMP")]
check(logged == received, "combined NMEA log is exactly the received stream (ignored tracker included)")

proc = [f for f in os.listdir(nmeadir) if f.endswith("_ID18157_Blue_Mavic_1_processed.nmea")]
check(len(proc) == 1, "processed NMEA file written: %s" % proc)
plines = [l.strip() for l in open(os.path.join(nmeadir, proc[0])) if l.strip()]
check(plines and all(nmea_ok(l) for l in plines), "every processed sentence has a valid checksum (%d lines)" % len(plines))
gga = [l for l in plines if l[3:6] == "GGA"][:60]; rmc = [l for l in plines if l[3:6] == "RMC"][:60]
check([g.split(",")[9] for g in gga] == exp, "processed GGA altitude = recomputed baro altitude")
orig_rmc = [l.split("*")[0] for b in blocks(cap)[:180] if b[0].startswith("$RFMSGFROM,18157") for l in b if l[3:6] == "RMC"]
check([r.split("*")[0] for r in rmc] == orig_rmc[:60], "processed RMC content unchanged")

haes = [e for e in cot.events if e["uid"] == "TRK.18157"]
check(haes and all(e["altsrc"] == "BARO" for e in haes), "18157 CoT uses barometric altitude")
c17 = [e for e in cot.events if e["uid"] == "TRK.17157"]
check(c17 and all(e["altsrc"] == "GPS" for e in c17), "17157 CoT still sent (record off) with GPS altitude")
check(not [e for e in cot.events if e["uid"] == "TRK.19157" and "Late add" not in (e["remarks"] or "")], "no CoT for 19157 while it was ignored")

assoc = glob.glob(os.path.join(csvdir, "*_associations.csv"))[0]; a = {r["Tracker_ID"]: r for r in csv.DictReader(open(assoc))}
check(a["17157"]["Record"] == "no" and a["18157"]["Altitude_Source"] == "recompute" and a["18157"]["NMEA_File"].endswith("_ID18157_Blue_Mavic_1.nmea"),
      "associations sidecar records the processing per tracker")
check.done()
