#!/usr/bin/env python3
"""
sitgs_download - pull logs off a SIT GPS tracker puck over USB
(replacement for the Windows "DownloadGPSLog" tool)

Protocol (recovered from sit_tracker_m0_lorarf_ubx_10 firmware):
  * within ~4 s of boot the puck listens on USB serial for the line  cmd
  * it answers  $SERIALCOM*  , mounts the SD card and prints a listing:
        $DIRLIST*
        $FILE,<name>,<YYYY-M-D H:M:S>,<bytes>*     (one per file)
        $DIRLISTEND*
  * commands (terminated by \\r or \\n):
        dir              listing as above
        type <file>      $FILELISTBOUNDARYSTART*  <raw bytes>  \\r\\n$FILELISTBOUNDARYEND*
        del <file>       $FILEDELSUCCESS*  or  $FILENOTFOUND*
        q                $EXIT*   (puck continues normal boot)

Usage:
  python3 sitgs_download.py                 # wait for a puck, list, download everything
  python3 sitgs_download.py --list          # just list
  python3 sitgs_download.py --since 2026-09-01 --dest logs/puck
  python3 sitgs_download.py --files GPSL0012.CSV GPSL0013.CSV
  python3 sitgs_download.py --delete-after  # delete each file from the SD after a verified download

Procedure: remove the battery, run this tool, then plug the puck's micro-USB into the Mac.
The tool waits for the new serial port and sends the handshake the moment it appears.
"""
import argparse, datetime as dt, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from apppaths import DATA_DIR
import serial, serial.tools.list_ports

HANDSHAKE_WINDOW_S = 6.0

def puck_ports():
    return {p.device for p in serial.tools.list_ports.comports() if (p.vid == 0x239A) or "usbmodem" in p.device or "ttyACM" in p.device}

def wait_for_new_port(existing, timeout=120):
    print("Waiting for the puck: remove the battery, then plug its micro-USB into this computer ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        new = puck_ports() - existing
        if new: return sorted(new)[0]
        time.sleep(0.05)
    return None

class Puck:
    def __init__(self, port, verbose=False):
        self.port = port; self.verbose = verbose; self.ser = None; self.buf = b""

    def open_and_handshake(self):
        t0 = time.time(); last_err = None
        while time.time() - t0 < HANDSHAKE_WINDOW_S:
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, 115200, timeout=0.05)
                self.ser.write(b"cmd\r\n"); self.ser.flush()
                if self._wait_for(b"$SERIALCOM*", 0.25): return True
            except (serial.SerialException, OSError) as e:
                last_err = e; self.ser = None; time.sleep(0.1)
        raise RuntimeError("no $SERIALCOM* reply within %.0fs (last error: %s). Remove the battery and re-plug USB, then retry." % (HANDSHAKE_WINDOW_S, last_err))

    def _read(self):
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if chunk:
            self.buf += chunk
            if self.verbose: sys.stderr.write(chunk.decode("ascii", "replace"))
        return chunk

    def _wait_for(self, token, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._read()
            i = self.buf.find(token)
            if i >= 0:
                self.buf = self.buf[i + len(token):]; return True
        return False

    def send(self, line):
        self.ser.write(line.encode("ascii") + b"\r\n"); self.ser.flush()

    def listing(self, timeout=15):
        """returns list of dicts name/size/mtime; the puck prints one automatically after the handshake"""
        if not self._wait_for(b"$DIRLIST*", timeout): raise RuntimeError("no $DIRLIST* from puck")
        t0 = time.time(); files = []
        while time.time() - t0 < timeout:
            self._read()
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1); line = line.strip().decode("ascii", "replace")
                if line.startswith("$DIRLISTEND*"): return files
                m = re.match(r"\$FILE,([^,]+),([^,]*),(\d+)\*", line)
                if m:
                    mt = None
                    try: mt = dt.datetime.strptime(m.group(2).strip(), "%Y-%m-%d %H:%M:%S")
                    except ValueError: pass
                    files.append({"name": m.group(1), "mtime": mt, "size": int(m.group(3))})
        raise RuntimeError("listing did not finish")

    def dir(self):
        self.buf = b""; self.send("dir"); return self.listing()

    def get(self, name, size, timeout=120):
        self.buf = b""; self.send("type " + name)
        if not self._wait_for(b"$FILELISTBOUNDARYSTART*", 5):
            if b"$FILENOTFOUND*" in self.buf: raise FileNotFoundError(name)
            raise RuntimeError("no boundary start for " + name)
        if self.buf.startswith(b"\r\n"): self.buf = self.buf[2:]
        elif self.buf.startswith(b"\n"): self.buf = self.buf[1:]
        end = b"$FILELISTBOUNDARYEND*"; t0 = time.time(); last = time.time(); n0 = 0
        while time.time() - t0 < timeout:
            if self._read(): last = time.time()
            i = self.buf.find(end)
            if i >= 0:
                data = self.buf[:i]; self.buf = self.buf[i + len(end):]
                if data.endswith(b"\r\n"): data = data[:-2]
                elif data.endswith(b"\n"): data = data[:-1]
                return data
            if len(self.buf) // 4096 != n0:
                n0 = len(self.buf) // 4096; sys.stdout.write("\r  %-16s %8d / %d bytes   " % (name, len(self.buf), size)); sys.stdout.flush()
            if time.time() - last > 5: break
        raise RuntimeError("transfer of %s stalled at %d bytes" % (name, len(self.buf)))

    def delete(self, name):
        self.buf = b""; self.send("del " + name)
        if self._wait_for(b"$FILEDELSUCCESS*", 5): return True
        return False

    def quit(self):
        try: self.send("q"); self._wait_for(b"$EXIT*", 2)
        except Exception: pass
        if self.ser: self.ser.close()

def logger_id_from(data):
    m = re.search(rb"\$LOGGERID,(\d+),", data)
    if m: return int(m.group(1))
    # CSV: header ... User_LoggerID is the 13th column
    for line in data.split(b"\n")[:50]:
        parts = line.split(b",")
        if len(parts) >= 13 and parts[0][:2] == b"20":
            try: return int(float(parts[12]))
            except ValueError: pass
    return None

def main():
    ap = argparse.ArgumentParser(description="Download logs from a SIT GPS tracker puck")
    ap.add_argument("--port", help="serial port (default: wait for a newly plugged puck)")
    ap.add_argument("--dest", default=os.path.join(DATA_DIR, "logs", "puck"))
    ap.add_argument("--list", action="store_true", help="list files only")
    ap.add_argument("--files", nargs="*", help="only these file names")
    ap.add_argument("--since", help="only files modified on/after this date (YYYY-MM-DD)")
    ap.add_argument("--delete-after", action="store_true", help="delete each file from the SD after a size-verified download")
    ap.add_argument("--verbose", action="store_true", help="echo the raw serial traffic")
    a = ap.parse_args()

    port = a.port or wait_for_new_port(puck_ports())
    if not port: print("no puck appeared"); return 1
    print("puck on", port)
    p = Puck(port, a.verbose)
    try:
        p.open_and_handshake(); print("handshake OK ($SERIALCOM*)")
        files = p.listing()
        since = dt.datetime.strptime(a.since, "%Y-%m-%d") if a.since else None
        sel = [f for f in files if (not a.files or f["name"] in a.files) and (not since or (f["mtime"] and f["mtime"] >= since))]
        print("%-16s %-20s %10s" % ("file", "modified", "bytes"))
        for f in files:
            print("%-16s %-20s %10d %s" % (f["name"], f["mtime"].strftime("%Y-%m-%d %H:%M:%S") if f["mtime"] else "-", f["size"], "" if f in sel else "(skipped)"))
        if a.list or not sel:
            return 0
        total = sum(f["size"] for f in sel); print("downloading %d files, %d bytes" % (len(sel), total))
        got = 0; lid_dir_cache = {}
        for f in sel:
            data = p.get(f["name"], f["size"])
            ok = len(data) == f["size"]
            lid = logger_id_from(data)
            d = os.path.join(a.dest, "ID%03d" % lid if lid is not None else "unknown-id"); os.makedirs(d, exist_ok=True)
            out = os.path.join(d, f["name"])
            with open(out, "wb") as fh: fh.write(data)
            print("\r  %-16s %8d bytes -> %s  %s      " % (f["name"], len(data), out, "OK" if ok else "SIZE MISMATCH (expected %d)" % f["size"]))
            got += 1
            if a.delete_after and ok:
                print("   deleted" if p.delete(f["name"]) else "   delete FAILED")
        print("done: %d files" % got)
    except Exception as e:
        print("ERROR:", e); return 1
    finally:
        p.quit()
    return 0

if __name__ == "__main__":
    sys.exit(main())
