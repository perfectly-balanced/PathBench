import sys, os
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner, TestResult

tests_path = os.path.dirname(os.path.abspath(__file__))
src_path = tests_path + "/../src"
sys.path.append(src_path)

if __name__ == '__main__':
    loader: TestLoader = unittest.TestLoader()
    tests: TestSuite = loader.discover(tests_path)
    testRunner: TextTestRunner = unittest.TextTestRunner()
    res: TestResult = testRunner.run(tests)
    sys.exit(0 if (len(res.errors) == 0 and len(res.failures) == 0) else 1)
