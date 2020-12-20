"""

The MVC pattern was cloned from https://github.com/wesleywerner/mvc-game-design

"""

import copy
import sys
import argparse

from algorithms.configuration.configuration import Configuration
from algorithms.lstm.trainer import Trainer
from analyzer.analyzer import Analyzer, available_algorithms
from generator.generator import Generator
from simulator.services.debug import DebugLevel
from simulator.services.services import Services, GenericServices
from simulator.simulator import Simulator

class MainRunner:
    main_services: Services

    def __init__(self, configuration: Configuration):
        self.main_services: Services = Services(configuration)
        self.run = self.main_services.debug.debug_func(DebugLevel.BASIC)(self.run)

    def run(self):
        if self.main_services.settings.generator:
            Generator.main(self)
        if self.main_services.settings.trainer:
            Trainer.main(self)
        if self.main_services.settings.analyzer:
            Analyzer.main(self)
        if self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()

        if self.main_services.settings.clear_cache:
            self.main_services.resources.cache_dir.clear()


def configure_and_run(args):
    config = Configuration()

    if args.list_algorithms:
        print("Available algorithms:")
        for key in available_algorithms.keys():
            print(f"  {key}")
        return True
    if args.analyzer_algorithms:
        config.analyzer_algorithms = args.analyzer_algorithms
        if not all(key in available_algorithms for key in config.analyzer_algorithms):
            invalid_algorithms = [key for key in config.analyzer_algorithms if not key in available_algorithms]
            invalid_str = ",".join('"' + a + '"' for a in invalid_algorithms)
            valid_str = ",".join('"' + a + '"' for a in available_algorithms)
            print(f"Invalid algorithm(s) specified: {invalid_str}")
            print(f"Available algorithms: {valid_str}")
            return False

    if args.visualiser:
        config.load_simulator = True
        config.simulator_graphics = True

    if args.generator:
        config.generator = True

    if args.analyzer or args.analyzer_algorithms: # analyzer_algorithms implies analyzer
        config.analyzer = True
    
    mr = MainRunner(config)
    mr.run()
    return True

def main() -> bool:
    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-v", "--visualiser", action='store_true', help="run simulator with graphics")
    parser.add_argument("-g", "--generator", action='store_true', help="run generator")
    parser.add_argument("-a", "--analyzer", action='store_true', help="run analyzer")
    parser.add_argument("--analyzer-algorithms", help="Select the algorithms to analyze", nargs="+")
    parser.add_argument("--list-algorithms", action="store_true", help="Print out a list of available algorithms")

    args = parser.parse_args()
    print("args:{}".format(args))
    return configure_and_run(args)

if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
