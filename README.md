# sitgs — SIT GPS-LoRa tracker ground station

Web reimplementation of the LabVIEW "LoRa GPS RX and Logger" client, tab for tab:
connection panel (Serial or Network), incoming data, message timeline and TDMA
transmission-slot plots, and the Barometric alt / CoT / Log / KML out / Overview /
Debug tabs. Every setting is live and saved to `sitgs.json`.

## Run (Mac)

    cd ~/Scensus\ CUAS\ and\ TE
    python3 sitgs.py

Open the address it prints (http://localhost:8090 unless that port is busy, then
8091). The receiver auto-connects on start; the Connect/Disconnect button and the
port picker work like the original. pyserial is bundled in `./serial`, Leaflet in
`./static`, so nothing needs installing.

Other ways to start it:

    python3 sitgs.py --list-ports
    python3 sitgs.py --record capture.nmea      # save raw serial bytes for replay
    python3 sitgs.py --replay capture.nmea      # no hardware; also from the Debug tab

## Tabs vs the LabVIEW client

**Connection** — Serial (port list + Refresh + baud) or Network (device-server IP +
TCP port, for the quad-channel PoE receiver). Green LED = connected. Counters for
lines, reports, bad checksums, bytes.

**Incoming data** — raw sentence stream. **Message timeline** — one row per tracker,
a tick per received report over the last 120 s. **Transmission time slots** — the
TDMA slot (0–19, 50 ms each) in which each report arrived; a healthy tracker draws a
flat line, a drifting clock or a collision shows as scatter.

**Barometric alt** — tracker grid (ID, RSSI bar, pressure, temp, GPS alt, baro alt,
age, battery, fix LED, sats); weather conditions; auto-update reference off or from
a stationary tracker at a known altitude; calculated sea-level pressure; disposition
Disable / As-is / Recompute; data age tolerance.

**Trackers** — the association list: one row per tracker ID (heard, or pre-registered
before an exercise with *Add tracker*) with its **target** (what it is mounted on),
callsign and **tactical symbol**. The symbol picker builds the CoT type from an
affiliation (friend / assumed friend / neutral / unknown / suspect / hostile / pending)
and a type (UAV rotary or fixed wing, aircraft, helicopter, ground unit / vehicle /
dismount, sensor, sea surface), with an Advanced box for any other CoT type string; the
preview is the MIL-STD-2525 symbol (milsymbol, bundled). A tracker's symbol goes out in
its CoT on every output feed; trackers left on *Default* use the CoT tab's default. The
target is added to CoT remarks and KML, and each session writes
`logs/csv/<session>_<event>_associations.csv` (ID, carrier, slot, target, callsign,
symbol, paired drone) next to the split CSVs. The Overview map draws the same symbols.

**CoT** — local geoid height and the value from GPS with auto-update; Emit CoT
toggle; default symbol (for trackers without their own); period; stale time; uid prefix and format
string; IncludeFlowTag; editable destination table (enable, IP, port, interface,
TTL — 224–239.x.x.x goes out as multicast) and the sink status pane.

**Log** — NMEA log folder with the live file name, Log data toggle and line count;
exercise name, split CSV folder, Enable split log toggle, row count and file names;
`$RXTIMESTAMP` injection. CSV columns match the manual §5.2 exactly.

**KML out** — write KML and GeoJSON files on an interval, or point Google Earth at
`/link.kml` (a network link that refreshes itself) with no file at all.

**Overview** — live map with trails (street or satellite basemap; needs internet for
tiles), trackers and drones drawn as their tactical symbols, and a summary of the
Trackers tab.

**Debug** — parser statistics, replay controls, clear trackers, event log.

## Files

    sitgs.py        backend (serial/TCP reader, parser, CSV, CoT, KML, web API)
    sitgs_ui.html   the UI
    sitgs.json      settings (created on first run; edit in the UI or by hand)
    serial/         bundled pyserial
    static/         bundled Leaflet
    logs/           nmea/, csv/, kml/ output

## Drones tab — Cosmostreamer boxes

The ground station is also a Cosmostreamer client, using the box's native UDP protocol
(port 5252, the one CosmoViewerNG uses). Boxes on the same network are discovered by
broadcast; others can be listed under **Cosmostreamer boxes** in the connection panel
(`ip` or `ip:port`), and boxes that publish through an MQTT broker (e.g. over a VPN) can
be read by enabling MQTT there.

The **Drones** tab lists every box with its type, licence, DJI link LED, drone model,
position, altitude above take-off, satellites, yaw, speed, batteries, flight mode and
in-flight/RTH flags. The **Tracker** column pairs a box with the puck mounted on that
drone. A paired drone is one CoT entity — the tracker's uid and callsign — whose position
comes from the DJI link while it is alive and falls back to the 915 MHz tracker when it
is not; the **Source** column shows which, and the Overview map draws the DJI position as
a diamond with a red line to the tracker's fix. The CoT tab's *Drones* block adds the
box's RTSP link to the CoT (ATAK/WinTAK open video from the icon) and controls whether
unpaired boxes are sent as `DJI.<name>`.

Altitude: a box reports height above take-off; the paired tracker's GPS altitude while
the drone is on the ground supplies the take-off MSL so the CoT carries a proper HAE.

### Testing without a box

    python3 cosmo_sim.py --port 5253 --name Alpha --lat 38.8447 --lon -77.0764 --link-drop 40:20

simulates a DJI Fly box: add `127.0.0.1:5253` to the manual host list, pair it with
your tracker, and watch Source flip DJI → LoRa → DJI when the simulated link drops at
40 s. Real boxes use port 5252 and need no manual entry.

Files: `cosmo.py` (client, MQTT reader), `cosmo_sim.py` (simulator).

## Mission tab — waypoint missions

Click the map to add waypoints, drag to move, set per-waypoint height/speed or use the
mission defaults (speed, height above take-off, RTH height, finish action, heading mode,
drone type). **Save** writes `logs/missions/<name>.json` and a DJI **.kmz** in WPML
format (the file DJI Fly/Pilot and Cosmostreamer use). **Upload to box** sends the KMZ to
the selected box through its own web upload (`waypoints.php`, documented in the firmware).

The *Fly it* buttons send the Cosmostreamer control strings (take/release control, start,
pause, resume, stop, RTH, land). Upload and RTH/land/control use commands found in the
firmware; the mission start/pause/stop strings are the PRO-licence commands and are
**unverified until tried on a real box** — they live in `sitgs.json → cosmo.mission_cmds`
so they can be corrected without touching code. Every command asks for confirmation.
The pilot's controller always overrides software control. Autonomous flight of several
aircraft is a regulatory matter (SFOC / Part 107 waiver) before it is a software one.

## Video tab — MediaMTX video wall

MediaMTX (https://github.com/bluenviron/mediamtx, single binary, `brew install mediamtx`
on a Mac) relays each box's RTSP stream to any number of viewers and records it. The
Video tab's *download mediamtx.yml* link generates a config with one path per discovered
box (source `rtsp://<box>:554/video`, recording on). Run `mediamtx mediamtx.yml` on the
video machine, set the WebRTC base URL (default `http://localhost:8889`), and the tab
shows one live WebRTC tile per box. RTSP for other tools: `rtsp://<video machine>:8554/<box name>`.
Ten 1080p boxes ≈ 150 Mbps on the LAN and ≈ 65 GB/hour recorded.

## Not yet

Quad-channel Ethernet receiver (needs its IP/port — use Network connection type),
RSSI/timeline history beyond 120 s, config-XML import from the LabVIEW install.

## Downloading logs from a puck (`sitgs_download.py`)

Replaces the Windows `DownloadGPSLog` tool. The puck's USB file-transfer protocol was
recovered from the firmware (`cmd` handshake within a few seconds of boot, then
`dir` / `type <file>` / `del <file>` / `q`).

    python3 sitgs_download.py            # wait for a puck, list, download everything
    python3 sitgs_download.py --list
    python3 sitgs_download.py --since 2026-09-01
    python3 sitgs_download.py --files GPSL0012.CSV
    python3 sitgs_download.py --delete-after   # delete from the SD after a size-verified copy

Procedure, same as the manual: remove the battery, start the tool, then plug the
puck's micro-USB in. Files land in `logs/puck/ID<logger id>/`, sorted by the logger
ID found inside each file. Several pucks can be done back to back — the tool waits
for each new USB port. Downloading takes ~40 s per MB over the puck's USB serial.
