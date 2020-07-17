"""

The MVC pattern was cloned from https://github.com/wesleywerner/mvc-game-design

"""

import copy

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
        #Two options, generator 
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
        #Three options, generator
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.analyzer:
            Generator.main(self)
            Trainer.main(self)
            Analyzer.main(self)
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Generator.main(self)
            Trainer.main(self)
        #Four options 
        elif self.main_services.settings.generator and self.main_services.settings.trainer and self.main_services.settings.analyzer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Generator.main(self)
            Trainer.main(self)
            Analyzer.main(self)
        #Trainer
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
        #Analyzer
        elif self.main_services.settings.analyzer and self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()
            Analyzer.main(self)
        #Singles
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

config = Configuration()
mr = MainRunner(config)
mr.run_multiple()
