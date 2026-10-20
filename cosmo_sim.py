#!/usr/bin/env python3
"""
cosmo_sim - pretend to be a Cosmostreamer box on the LAN (native UDP protocol, port 5252)

  python3 cosmo_sim.py                       # one box "SIM-1", DJI Fly type, drone circling near Ottawa
  python3 cosmo_sim.py --name Alpha --lat 38.8447 --lon -77.0764 --port 5252
  python3 cosmo_sim.py --port 5253 --name Bravo --radius 0.002 --link-drop 40:20   # drop the DJI link at t=40s for 20s
  python3 cosmo_sim.py --no-drone                # a box with no drone/controller attached: status packets only
  python3 cosmo_sim.py --replay logs/cosmo/2026_10_20_10_00_00_cosmo.jsonl   # play back a capture from a real box

Several boxes on one computer: run several copies on different ports and list them in the
ground station's manual host list as 127.0.0.1:5253 etc. (real boxes always use 5252).
"""
import argparse, json, math, socket, struct, time

START = 0x52
def i16(v): return struct.pack("<h", int(max(-32768, min(32767, v))))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5252); ap.add_argument("--name", default="SIM-1")
    ap.add_argument("--app-type", type=int, default=22); ap.add_argument("--lat", type=float, default=45.3831); ap.add_argument("--lon", type=float, default=-75.7020)
    ap.add_argument("--radius", type=float, default=0.003, help="circle radius in degrees"); ap.add_argument("--period", type=float, default=90, help="seconds per lap")
    ap.add_argument("--link-drop", default="", help="start:duration seconds, e.g. 40:20 — simulates a jammed DJI link")
    ap.add_argument("--sn", default="1581F5SIM0001")
    ap.add_argument("--no-drone", action="store_true", help="box with nothing attached: answers discovery, sends status only")
    ap.add_argument("--replay", help="capture .jsonl (sitgs --cosmo-capture / --cosmo-probe): replay what the real box sent")
    ap.add_argument("--replay-peer", help="which box in the capture (host:port); default: the first that answered discovery")
    a = ap.parse_args()
    rec = load_replay(a.replay, a.replay_peer) if a.replay else None
    drop = None
    if a.link_drop:
        s, d = a.link_drop.split(":"); drop = (float(s), float(d))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", a.port)); sock.settimeout(0.2)
    clients = {}; counter = 0; t0 = time.time(); last_push = 0
    print("cosmo_sim '%s' listening on UDP %d (app type %d)" % (a.name, a.port, a.app_type))

    def send(payload, addr):
        nonlocal counter
        sock.sendto(struct.pack("<BBHII", START, 0, len(payload), counter, 0) + payload, addr); counter += 1

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            if len(data) >= 13 and data[0] == START:
                p = data[12:]
                if p[0] == 0:      # discovery
                    if rec: sock.sendto(rec["answer"], addr)
                    else:
                        ans = bytes([1, a.app_type, 1, len(clients)]) + a.name.encode().ljust(40, b"\0") + bytes([0, 24, 11])
                        send(ans, addr)
                elif p[0] == 2:    # alive
                    clients[addr] = time.time()
                elif p[0] == 3:
                    clients.pop(addr, None)
                elif p[0] in (0x21, 0x7b):  # '!' or '{' text command
                    print("command from %s: %s" % (addr, p.decode("ascii", "replace")))
        except socket.timeout:
            pass
        now = time.time()
        for c in [c for c, t in clients.items() if now - t > 4]: clients.pop(c)
        if rec:
            if clients: replay_due(rec, now, lambda d: [sock.sendto(d, c) for c in list(clients)])
            continue
        if now - last_push >= 1.0 and clients and a.no_drone:
            last_push = now
            for c in list(clients): send(bytes([4, a.app_type, 1, len(clients)]), c)
            continue
        if now - last_push >= 1.0 and clients:
            last_push = now; t = now - t0
            linked = not (drop and drop[0] <= t < drop[0] + drop[1])
            ang = 2 * math.pi * t / a.period
            lat = a.lat + a.radius * math.sin(ang); lon = a.lon + a.radius * math.cos(ang) * 1.4
            alt = 60 + 30 * math.sin(ang * 2); yaw = (math.degrees(ang) + 90) % 360; hspeed = 2 * math.pi * a.radius * 111000 / a.period
            if linked:
                text = ("camera_detected=1,camera_model=DJI Mini 4 Pro,drone_sn=%s,battery_level=%d,voltage=15.2,rc_battery_level=88,"
                        "gps_fix=3,gps_sat_count=14,gps_lat=%.7f,gps_lng=%.7f,alt=%.1f,gps_home_lat=%.7f,gps_home_lng=%.7f,"
                        "gps_to_lat=%.7f,gps_to_lng=%.7f,home_distance=%.0f,home_heading=%.0f,flight_mode=Normal,in_flight=1,landing=0,rth=0,"
                        "ftime_left=%d,m_alt=120,m_dist=500,rth_alt=50,ctrl_app_en=0,ctrl_rc_en=1,tx_mode=2") % (
                        a.sn, max(5, 100 - int(t / 12)), lat, lon, alt, a.lat, a.lon, a.lat, a.lon,
                        a.radius * 111000, (yaw + 180) % 360, max(0, 1200 - int(t)))
                b = bytearray(56); b[0] = 6; b[1] = 0
                b[3:5] = i16(0); b[5:7] = i16(-300); b[7:9] = i16(0)
                b[37:39] = i16(yaw * 10); b[39:41] = i16(50); b[41:43] = i16(-20)
                b[51:53] = i16(hspeed * 10); b[53:55] = i16(0)
            else:
                text = "camera_detected=0,camera_model=DJI Mini 4 Pro,drone_sn=%s,gps_lst_lat=%.7f,gps_lst_lng=%.7f" % (a.sn, lat, lon)
                b = None
            for c in list(clients):
                send(bytes([4, a.app_type, 1, len(clients)]), c)
                send(bytes([7]) + text.encode(), c)
                if b: send(bytes(b), c)
            print("\r t=%4.0fs  link=%s  lat=%.5f lon=%.5f alt=%.0f yaw=%.0f  clients=%d   " % (t, "UP " if linked else "DOWN", lat, lon, alt, yaw, len(clients)), end="", flush=True)

def load_replay(path, peer=None):
    """Datagrams the box sent (dir rx) in a capture, with their timing, sent back verbatim."""
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rx = [r for r in rows if r["dir"] == "rx"]
    answers = [r for r in rx if len(bytes.fromhex(r["hex"])) > 12 and bytes.fromhex(r["hex"])[12] == 1]
    if not answers: raise SystemExit("no discovery answer in %s" % path)
    peer = peer or answers[0]["peer"]
    mine = [r for r in rx if r["peer"] == peer]
    stream = [(r["t"] - mine[0]["t"], bytes.fromhex(r["hex"])) for r in mine if bytes.fromhex(r["hex"])[12:13] != b"\x01"]
    print("replaying %d datagrams from %s (%.0f s)" % (len(stream), peer, stream[-1][0] if stream else 0))
    return {"answer": bytes.fromhex(next(r for r in answers if r["peer"] == peer)["hex"]), "stream": stream, "i": 0, "t0": None}

def replay_due(rec, now, send):
    if not rec["stream"]: return
    if rec["t0"] is None: rec["t0"] = now
    while rec["stream"][rec["i"]][0] <= now - rec["t0"]:
        send(rec["stream"][rec["i"]][1]); rec["i"] += 1
        if rec["i"] >= len(rec["stream"]): rec["i"] = 0; rec["t0"] = now; break   # loop

if __name__ == "__main__":
    main()
