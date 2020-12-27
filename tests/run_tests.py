import sys
import os
import argparse
import subprocess
import atexit

import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult

def kill_processes() -> None:
    global g_procs

    for p in g_procs:
        p.kill()
        p.communicate()

    g_procs = []

def launch_process(cmd) -> None:
    global g_procs

    print(" ".join(cmd))
    g_procs.append(subprocess.Popen(cmd))

def setup(args) -> None:
    if args.headless:
        global g_procs
        g_procs = []

        atexit.register(kill_processes)

        launch_process(["Xvfb", args.display, "-screen", "0", "2112x1376x24", "-fbdir", "/var/tmp"])
        os.environ["DISPLAY"] = args.display

        if args.view_display:
            launch_process(["x11vnc", "-display", args.display, "-localhost"])
            launch_process(["vncviewer", "-display", ":0"])

def run() -> bool:
    tests_path = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(os.path.dirname(tests_path), "src")
    sys.path.append(src_path)

    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(tests_path)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    return (len(res.errors) == 0 and len(res.failures) == 0)

def main() -> bool:
    parser = argparse.ArgumentParser(prog="run_tests.py",
                                     description="PathBench test runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--headless", action="store_true", help="spawn a virtual display and use it for testing graphics")
    parser.add_argument("--display", default=":99", help="spawned virtual display identifier")
    parser.add_argument("--view-display", action="store_true", help="open an interactive view of the spawned virtual display")
    parser.add_argument('test_args', metavar='TEST_ARG', nargs='*', help="arguments for test framework")

    args = parser.parse_args()
    print("args:{}".format(args))

    # remove all custom arguments for testing framework
    for a in ("--headless", "--display", "--view-display"):
        for i in range(1, len(sys.argv)):
            s = sys.argv[i]
            if s.startswith(a):
                if len(s) == len(a):
                    sys.argv.pop(i)
                    if len(sys.argv) > i and not sys.argv[i].startswith("-"):
                        sys.argv.pop(i)  # pop value
                    break
                else:
                    sys.argv.pop(i)
                    break

    setup(args)
    return run()


if __name__ == '__main__':
    res = main()
    sys.exit(int(not res))
