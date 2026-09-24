"""Resource vs data roots, source-run and PyInstaller-frozen.

RES_DIR  read-only assets bundled with the program (sitgs_ui.html, static/).
DATA_DIR everything the program writes (sitgs.json, logs/**, recordings) and
         where relative replay paths resolve. Frozen: the folder holding the
         executable when writable, else a per-user app-data dir. From source:
         the script folder, exactly as before.
"""
import os, sys, uuid

FROZEN = getattr(sys, "frozen", False)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

RES_DIR = getattr(sys, "_MEIPASS", _SCRIPT_DIR) if FROZEN else _SCRIPT_DIR

def _writable(d):
    try:
        probe = os.path.join(d, ".sitgs-write-%s" % uuid.uuid4().hex[:8])
        with open(probe, "w"): pass
        os.remove(probe)
        return True
    except OSError:
        return False

def _data_dir():
    if not FROZEN:
        return _SCRIPT_DIR
    exe_dir = os.path.dirname(os.path.abspath(sys.executable))
    if _writable(exe_dir):
        return exe_dir
    if sys.platform == "win32":
        base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
        d = os.path.join(base, "sitgs")
    elif sys.platform == "darwin":
        d = os.path.expanduser("~/Library/Application Support/sitgs")
    else:
        d = os.path.expanduser("~/.local/share/sitgs")
    os.makedirs(d, exist_ok=True)
    return d

DATA_DIR = _data_dir()
