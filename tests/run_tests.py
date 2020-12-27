import sys, os
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult
import subprocess
import atexit

tests_path = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(os.path.dirname(tests_path), "src")
sys.path.append(src_path)

g_disp_proc = None

def kill_virtual_display() -> None:
    global g_disp_proc
    if g_disp_proc is None:
        return

    g_disp_proc.kill()
    g_disp_proc.communicate()
    g_disp_proc = None

    atexit.unregister(kill_virtual_display)

cmd = ["Xvfb", ":99", "-screen", "0", "2112x1376x24", "-fbdir", "/var/tmp"]
print(" ".join(cmd))
g_disp_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
atexit.register(kill_virtual_display)

os.environ["PATH_BENCH_FULLSCREEN"] = "PATH_BENCH_FULLSCREEN"
os.environ["DISPLAY"] = ":99"

if __name__ == '__main__':
    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(tests_path)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    sys.exit(0 if (len(res.errors) == 0 and len(res.failures) == 0) else 1)
