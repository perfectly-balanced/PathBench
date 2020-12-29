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
    
    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(tests_path)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    return (len(res.errors) == 0 and len(res.failures) == 0)

if __name__ == '__main__':
    res = run()
    sys.exit(int(not res))
