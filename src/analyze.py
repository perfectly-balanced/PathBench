from algorithms.configuration.configuration import Configuration
from simulator.services.debug import DebugLevel
from main import MainRunner
from analyzer.analyzer import Analyzer


def main():
    c: Configuration = Configuration()

    # set configuration params
    c.simulator_write_debug_level = DebugLevel.HIGH

    main_runner = MainRunner(c)
    analyzer = Analyzer(main_runner.main_services)
    analyzer.analyze_algorithms()

if __name__== '__main__':
    main()
