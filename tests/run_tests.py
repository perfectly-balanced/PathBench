import sys
import os
import argparse
import atexit
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult

from utils import make_src_modules_importable

def run() -> bool:
    # add src folder to system path
    make_src_modules_importable()

    tests_path = os.path.dirname(os.path.abspath(__file__))
    
    parser = argparse.ArgumentParser(prog="run_tests.py",
                                     description="PathBench test runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--discovery-path", default=tests_path, help="recursively discovers tests using the specified file path")
    parser.add_argument("--discovery-pattern", default='test*.py', help="regex expression with discovered tests that have a matching file path being executed")

    args, sys.argv = parser.parse_known_args()
    print("args:{}".format(args))

    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(args.discovery_path, args.discovery_pattern)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    return (len(res.errors) == 0 and len(res.failures) == 0)

if __name__ == '__main__':
    res = run()
    sys.exit(int(not res))
