#!/usr/bin/env python3
"""Generate a synthetic receiver capture (sample.nmea) for --replay testing.
Two trackers circling near Ottawa, 1 report/s each, using the exact sentence
shapes the SITGPSLoRaRFRX firmware emits."""
import math, sys

def cks(body):
    x = 0
    for ch in body: x ^= ord(ch)
    return "$%s*%02X" % (body, x)

def dm(deg):
    d = int(abs(deg)); m = (abs(deg) - d) * 60
    return d, m

out = []
n = int(sys.argv[1]) if len(sys.argv) > 1 else 120
for i in range(n):
    for tid, r, lat0, lon0 in ((23, 0.004, 45.3831, -75.7020), (100, 0.0, 45.3900, -75.7100)):
        a = i * 2 * math.pi / 60
        lat = lat0 + r * math.sin(a); lon = lon0 + r * math.cos(a) * 1.4
        alt = 80 + 40 * math.sin(a) if r else 20
        hh, mm, ss = 10, 39 + i // 60, i % 60
        dlat, mlat = dm(lat); dlon, mlon = dm(lon)
        gga = "GPGGA,%02d%02d%02d.000,%02d%07.4f,N,%03d%07.4f,W,1,08,01.2,%.1f,M,-34.2,M,0,0" % (hh, mm, ss, dlat, mlat, dlon, mlon, alt)
        rmc = "GPRMC,%02d%02d%02d.000,A,%02d%07.4f,N,%03d%07.4f,W,%04.1f,%05.1f,290317,,,D" % (hh, mm, ss, dlat, mlat, dlon, mlon, 6.0 if r else 0.0, (math.degrees(a) + 90) % 360)
        p = 101325 * (1 - 0.0065 * alt / 288.15) ** 5.257
        out += [
            "$RFMSGFROM,16%03d,43,111,0,0*" % tid,
            "$DECOMPRESS,63*",
            "$BAROALT,%.2f,15.13,%.2f*" % (p, alt - 1),
            cks(gga), cks(rmc),
            "$BATMV,%d*" % (4000 - i),
            "$HRFSSI,%d*" % (-40 - (i % 30)),
            "$RFMSGEND*",
        ]
open("sample.nmea", "w").write("\n".join(out) + "\n")
print("wrote sample.nmea with", n, "reports x 2 trackers")
