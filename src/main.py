import sys
import argparse
import re

from panda3d.core import load_prc_file_data
from screeninfo import get_monitors

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
        if self.main_services.settings.trainer:
            Trainer.main(self)
        if self.main_services.settings.analyzer:
            Analyzer.main(self)
        if self.main_services.settings.load_simulator:
            simulator: Simulator = Simulator(self.main_services)
            simulator.start()

        if self.main_services.settings.clear_cache:
            self.main_services.resources.cache_dir.clear()


def configure_and_run(args) -> bool:
    config = Configuration()

    if args.visualiser:
        config.load_simulator = True
        config.simulator_graphics = True

        if args.visualiser_flags is not None:
            integral_pair_re = re.compile("\s*\(\s*[0-9]+\s*,\s*[0-9]+\s*\)\s*")
            naked_integral_pair_re = re.compile("\s*[0-9]+\s*[0-9]+\s*")

            data = ""

            def use_display_resolution():
                nonlocal data

                for m in get_monitors():
                    data += "win-size {} {}\n".format(m.width, m.height)
                    break

            for f in args.visualiser_flags:
                if f.strip().lower() == "windowed-fullscreen":
                    use_display_resolution()
                    data += "win-origin 0 0"
                    data += "undecorated true"
                    continue

                n, v = f.split("=")
                n = n.strip().lower()
                v = v.strip().lower()
                if v in ("true", "1", "yes"):
                    data += "{} true\n".format(n)
                elif v in ("false", "0", "no"):
                    data += "{} false\n".format(n)
                elif integral_pair_re.match(v) is not None:
                    v = re.sub('{\s+}|\(|\)', '', v)
                    v1, v2 = v.split(',')
                    data += "{} {} {}\n".format(n, v1, v2)
                elif naked_integral_pair_re.match(v) is not None:
                    v = v.strip()
                    v = re.sub('\s+', ',', v)
                    v1, v2 = v.split(',')
                    data += "{} {} {}\n".format(n, v1, v2)
                else:
                    data += "{} {}".format(n, v)

            if "fullscreen true" in data and "win-size" not in data:
                use_display_resolution()

            load_prc_file_data('', data)

    elif args.visualiser_flags:
        raise ValueError("Visualiser isn't running, but flags were provided")

    if args.generator:
        config.generator = True

    mr = MainRunner(config)
    mr.run()
    return True

def main() -> bool:
    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-v", "--visualiser", action='store_true', help="run simulator with graphics")
    parser.add_argument("-g", "--generator", action='store_true', help="run generator")
    parser.add_argument("-V", dest="visualiser_flags", metavar="VISUALISER_FLAG", action='append',
                        help="Visualiser options (overriding Panda3D's default Config.prc - see https://docs.panda3d.org/1.10/python/programming/configuration/configuring-panda3d#configuring-panda3d for options [windowed-fullscreen is an additional custom option])")

    args = parser.parse_args()
    print("args:{}".format(args))
    return configure_and_run(args)


if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
