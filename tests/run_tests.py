import sys
import os
import argparse
import atexit
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult

def run() -> bool:
    tests_path = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(os.path.dirname(tests_path), "src")
    sys.path.append(src_path)

    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(tests_path)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    return (len(res.errors) == 0 and len(res.failures) == 0)

if __name__ == '__main__':
    res = run()
    sys.exit(int(not res))
