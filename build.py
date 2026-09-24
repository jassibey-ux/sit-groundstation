#!/usr/bin/env python3
"""Build shippable sitgs binaries with PyInstaller and stage a release zip.

Run from the repo root, with requirements-build.txt installed:

    python build.py [--version 0.1.0]

Produces dist/sitgs(.exe), dist/sitgs-download(.exe) and
release/sitgs-<version>-<os>-<arch>.zip containing the two binaries,
capture.nmea and TESTING.md.

The vendored ./serial (pyserial 3.5) is what gets bundled — same code that
from-source runs use. pyserial loads its platform backend and its
serial_for_url("socket://...") handlers via importlib, which PyInstaller's
static analysis cannot see, hence the explicit hidden imports below.
"""
import argparse, os, platform, shutil, subprocess, sys, zipfile

HERE = os.path.dirname(os.path.abspath(__file__))

HIDDEN = ["serial.urlhandler.protocol_socket", "serial.urlhandler.protocol_rfc2217",
          "serial.urlhandler.protocol_loop", "serial.tools.list_ports"]
if sys.platform == "win32":
    HIDDEN += ["serial.serialwin32", "serial.tools.list_ports_windows"]
else:
    HIDDEN += ["serial.serialposix", "serial.tools.list_ports_posix"]
    if sys.platform == "darwin":
        HIDDEN += ["serial.tools.list_ports_osx"]
    else:
        HIDDEN += ["serial.tools.list_ports_linux"]

def run_pyinstaller(script, name, add_data=()):
    cmd = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--onefile",
           "--console", "--name", name,
           "--paths", HERE,
           "--distpath", os.path.join(HERE, "dist"),
           "--workpath", os.path.join(HERE, "build"),
           "--specpath", os.path.join(HERE, "build")]
    for h in HIDDEN:
        cmd += ["--hidden-import", h]
    for src, dst in add_data:
        cmd += ["--add-data", "%s%s%s" % (os.path.join(HERE, src), os.pathsep, dst)]
    cmd.append(os.path.join(HERE, script))
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=HERE)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default=os.environ.get("SITGS_VERSION", "dev"))
    args = ap.parse_args()

    for d in ("dist", "build", "release"):
        shutil.rmtree(os.path.join(HERE, d), ignore_errors=True)

    run_pyinstaller("sitgs.py", "sitgs",
                    add_data=[("sitgs_ui.html", "."), ("static", "static")])
    run_pyinstaller("sitgs_download.py", "sitgs-download")

    exe = ".exe" if os.name == "nt" else ""
    osname = {"win32": "windows", "darwin": "macos"}.get(sys.platform, "linux")
    arch = platform.machine().lower().replace("amd64", "x64").replace("x86_64", "x64")
    stage = os.path.join(HERE, "release", "sitgs-%s" % args.version)
    os.makedirs(stage, exist_ok=True)
    for f in ("sitgs" + exe, "sitgs-download" + exe):
        shutil.copy2(os.path.join(HERE, "dist", f), stage)
    for f in ("capture.nmea", "TESTING.md"):
        shutil.copy2(os.path.join(HERE, f), stage)

    zpath = os.path.join(HERE, "release", "sitgs-%s-%s-%s.zip" % (args.version, osname, arch))
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(stage)):
            # preserve the executable bit for the mac/linux binaries
            z.write(os.path.join(stage, f), f)
            zi = z.infolist()[-1]
            zi.external_attr = (os.stat(os.path.join(stage, f)).st_mode & 0xFFFF) << 16
    print("release zip:", zpath)
    return 0

if __name__ == "__main__":
    sys.exit(main())
