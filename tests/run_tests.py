import sys
import os
import argparse
import atexit
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult

from utils import handle_display_args, kill_processes, remove_custom_flags

def setup(args) -> None:
    atexit.register(kill_processes)
    handle_display_args(args)

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
    parser.add_argument("--spawn-display", nargs='?', default=None, const=":99", help="spawn a virtual display for testing graphics and sets $DISPLAY")
    parser.add_argument("--view-display", nargs='?', default=None, const="auto", help="open an interactive view of a virtual display (defaults to view $DISPLAY)")

    args, sys.argv = parser.parse_known_args()
    print("args:{}".format(args))

    setup(args)
    return run()


if __name__ == '__main__':
    res = main()
    sys.exit(int(not res))
