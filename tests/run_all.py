"""Run every tests/test_*.py in turn; exit non-zero if any fails. Used by CI before building."""
import glob, os, subprocess, sys, time

here = os.path.dirname(os.path.abspath(__file__)); failed = []
for t in sorted(glob.glob(os.path.join(here, "test_*.py"))):
    name = os.path.basename(t); t0 = time.time()
    print("=== %s" % name, flush=True)
    rc = subprocess.call([sys.executable, t])
    print("=== %s %s (%.0f s)\n" % (name, "ok" if rc == 0 else "FAILED", time.time() - t0), flush=True)
    if rc: failed.append(name)
print("ALL TESTS PASSED" if not failed else "FAILED: %s" % ", ".join(failed))
sys.exit(1 if failed else 0)
