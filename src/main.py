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


