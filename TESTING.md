# sitgs — tester quick start

sitgs replaces the LabVIEW "LoRa GPS RX and Logger" client. This zip contains
everything; nothing to install.

    sitgs / sitgs.exe            the ground station (web UI)
    sitgs-download / .exe        puck SD-log downloader (replaces DownloadGPSLog)
    capture.nmea                 a recorded session for hardware-free replay
    TESTING.md                   this file

## Windows

1. Unzip to any writable folder (Desktop, Documents — not Program Files).
2. Double-click `sitgs.exe`. SmartScreen will warn because the build is
   unsigned: click **More info → Run anyway** (expected for a test build).
3. A console window opens and prints `UI: http://localhost:8090` — open that
   address in a browser.

## Mac

1. Unzip to any folder.
2. First launch: right-click `sitgs` → **Open** → **Open** (unsigned build),
   or in Terminal: `xattr -d com.apple.quarantine ./sitgs && ./sitgs`
3. Open the printed address (http://localhost:8090) in a browser.

## 5-minute smoke test (no hardware needed)

1. Start `sitgs`, open http://localhost:8090.
2. **Debug** tab → "Replay a recording" → file `capture.nmea` → **Replay**.
3. Tracker **157** appears: check the message timeline, TDMA slot plot,
   Barometric alt grid, and the **Overview** map (trail should draw; the
   street basemap needs internet, the tracker table works without).
4. **Log** tab: confirm NMEA + split CSV are being written — the console
   printed the data folder at startup (normally the folder you unzipped to);
   look in `logs/nmea/` and `logs/csv/`.
5. **CoT** tab: destinations are configurable; if you have ATAK/WinTAK on the
   LAN, point a destination at it and confirm icons.
6. Settings persist in `sitgs.json` next to the binaries — change something in
   the UI, restart, confirm it stuck.

## With hardware

- Plug the 915 MHz receiver in, **Connection** panel → Refresh → Connect
  (auto-connect is on by default; baud 115200).
- Puck log download: remove the puck battery, run `sitgs-download`, plug the
  puck's micro-USB in. Files land in `logs/puck/ID<logger id>/`.

## Viewing from another machine (tablet etc.)

Run with `--host` open: the server already listens on all interfaces; just
browse to `http://<this machine's IP>:8090` from the other device. Allow the
app through the OS firewall if prompted.

## Tracker IDs

Trackers are shown by their full over-the-air ID, e.g. **17005** = carrier
917 MHz, slot 5; **18005** = 918 MHz, slot 5. The Barometric alt grid and the
Overview table have Carrier and Slot columns; map labels read
`17005 · 917 MHz · slot 5`. CoT uids are `TRK.17005` and split CSV files are
`…_ID17005.csv`. Callsigns and drone pairings saved under an old short ID
(e.g. "5") move to the full ID automatically the first time that tracker is
heard.

## If the app ever looks frozen

1. Press **Enter** in the console window once.
2. Open http://localhost:8090 in the browser: does the page still update?
   Is the uptime counter moving while lines/reports are stuck?
3. Send us `logs/sitgs-events.log` from the data folder. It has a heartbeat
   line every minute showing which part stopped, plus any watchdog actions
   (a stuck serial read is cancelled and reconnected automatically after
   ~10 s).

## Known limitations of this test build

- Unsigned binaries (SmartScreen/Gatekeeper warnings above are expected).
- Mac build is Apple Silicon; on an Intel Mac run from source instead
  (`python3 sitgs.py` — needs only stock Python 3, no pip installs).
- Cosmostreamer mission start/pause/stop commands are unverified on real
  hardware (PRO-licence strings recovered from firmware).

Please note anything broken or different from the LabVIEW client, with the
tab name and what you did.
