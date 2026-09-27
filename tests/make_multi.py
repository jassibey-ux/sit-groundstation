"""Synthetic receiver stream for N trackers across two bands (ids 17001.., 18001..), 1 report/s each,
each flying its own circle. Used by the many-tracker and soak tests.

    python tests/make_multi.py --trackers 40 --seconds 120 --out multi.nmea
"""
import argparse, math, time

def _cs(body):
    x = 0
    for ch in body: x ^= ord(ch)
    return "$%s*%02X" % (body, x)

def tracker_ids(n):
    half = (n + 1) // 2
    return [17001 + i for i in range(half)] + [18001 + i for i in range(n - half)]

def report(tid, t, lat0=38.8447, lon0=-77.0764):
    k = tid % 1000 + (tid // 1000) * 7
    a = t / 60.0 * 2 * math.pi + k
    lat = lat0 + 0.002 * math.sin(a) + 0.0004 * (k % 9); lon = lon0 + 0.003 * math.cos(a) + 0.0004 * (k % 7)
    alt = 60 + 20 * math.sin(a / 2) + k % 30
    pa = 101325.0 * (1 - 2.25577e-5 * alt) ** 5.25588
    tm = time.gmtime(t); hms = time.strftime("%H%M%S", tm) + ".%03d" % int((t % 1) * 1000); dmy = time.strftime("%d%m%y", tm)
    def dm(v, pos, neg, w):
        s = pos if v >= 0 else neg; v = abs(v); d = int(v)
        return ("%0" + str(w) + "d%07.4f") % (d, (v - d) * 60), s
    la, ns = dm(lat, "N", "S", 2); lo, ew = dm(lon, "E", "W", 3)
    return ["$RFMSGFROM,%d,35,111,0,0*" % tid, "$DECOMPRESS,70*", "$BAROALT,%.2f,26,%.1f*" % (pa, alt), "$BATMV,%d*" % (3600 + k % 150),
            _cs("GPGGA,%s,%s,%s,%s,%s,1,09,01.1,%.1f,M,-33.3,M,," % (hms, la, ns, lo, ew, alt)),
            _cs("GPRMC,%s,A,%s,%s,%s,%s,012.3,090.0,%s,," % (hms, la, ns, lo, ew, dmy)),
            "$HRFSSI,%d*" % (-50 - k % 40), "$RFMSGEND*"]

def second(ids, t):
    lines = []
    for tid in ids: lines += report(tid, t)
    return lines

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--trackers", type=int, default=40)
    ap.add_argument("--seconds", type=int, default=60); ap.add_argument("--out", required=True)
    a = ap.parse_args(); ids = tracker_ids(a.trackers); t0 = time.time()
    with open(a.out, "w") as f:
        for s in range(a.seconds): f.write("\n".join(second(ids, t0 + s)) + "\n")
    print("%d trackers x %d s -> %s (replay with --speed %d for 1 Hz each)" % (a.trackers, a.seconds, a.out, a.trackers))
