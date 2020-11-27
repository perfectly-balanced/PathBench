"""

The MVC pattern was cloned from https://github.com/wesleywerner/mvc-game-design

"""

import copy
import sys
import argparse

from algorithms.configuration.configuration import Configuration
from algorithms.lstm.trainer import Trainer
from analyzer.analyzer import Analyzer
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
        elif self.main_services.settings.trainer:
            Trainer.main(self)
        elif self.main_services.settings.analyzer:
            Analyzer.main(self)
        elif self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()

        if self.main_services.settings.clear_cache:
            self.main_services.resources.cache_dir.clear()


#Work in progress

    def run_multiple(self):
        # Two options, generator
        if self.main_services.settings.generator and self.main_services.settings.trainer:
            Generator.main(self)
            Trainer.main(self)
        elif self.main_services.settings.generator and self.main_services.settings.analyzer:
            Generator.main(self)
            Analyzer.main(self)
        elif self.main_services.settings.generator and self.main_services.settings.load_simulator:
            Generator.main(self)
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
        # Three options, generator
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.analyzer:
            Generator.main(self)
            Trainer.main(self)
            Analyzer.main(self)
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Generator.main(self)
            Trainer.main(self)
        # Four options
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.analyzer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Generator.main(self)
            Trainer.main(self)
            Analyzer.main(self)
        # Trainer
        elif self.main_services.settings.trainer and self.main_services.settings.analyzer:
            Trainer.main(self)
            Analyzer.main(self)
        elif self.main_services.settings.trainer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Trainer.main(self)
        elif self.main_services.settings.trainer and self.main_services.settings.analyzer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Trainer.main(self)
            Analyzer.main(self)
        # Analyzer
        elif self.main_services.settings.analyzer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Analyzer.main(self)
        # Singles
        elif self.main_services.settings.generator:
            Generator.main(self)
        elif self.main_services.settings.trainer:
            Trainer.main(self)
        elif self.main_services.settings.analyzer:
            Analyzer.main(self)
        elif self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()

        if self.main_services.settings.clear_cache:
            self.main_services.resources.cache_dir.clear()

def configure_and_run(args) -> bool:
    config = Configuration()

    if args.visualiser:
        config.load_simulator = True
        config.simulator_graphics = True
    
    if args.trainer:
        config.trainer = True

    if args.generator:
        config.generator = True

    if args.debug == 'NONE':
        config.simulator_write_debug_level = DebugLevel.NONE
    elif args.debug == 'BASIC':
        config.simulator_write_debug_level = DebugLevel.BASIC
    elif args.debug == 'LOW':
        config.simulator_write_debug_level = DebugLevel.LOW
    elif args.debug == 'MEDIUM':
        config.simulator_write_debug_level = DebugLevel.MEDIUM
    elif args.debug == 'HIGH':
        config.simulator_write_debug_level = DebugLevel.HIGH
    else:
        raise NotImplementedError
    
    mr = MainRunner(config)
    mr.run_multiple()
    return True

def main() -> bool:
    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-v", "--visualiser", action='store_true', help="run simulator with graphics")
    parser.add_argument("-g", "--generator", action='store_true', help="run generator")
    parser.add_argument("-t", "--trainer", action='store_true', help="runs model trainer")
    parser.add_argument("-d", "--debug", choices=['NONE', 'BASIC', 'LOW', 'MEDIUM', 'HIGH'], default='HIGH', help="set the debug level when running, default is high")

    args = parser.parse_args()
    print("args:{}".format(args))
    return configure_and_run(args)

if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
