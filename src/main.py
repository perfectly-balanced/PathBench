import copy
import sys
import os
import argparse
import re

from panda3d.core import load_prc_file_data
from screeninfo import get_monitors

from algorithms.configuration.configuration import Configuration
from algorithms.algorithm_manager import AlgorithmManager
from algorithms.lstm.trainer import Trainer
from analyzer.analyzer import Analyzer
from generator.generator import Generator
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
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

def arg_valid(attr, args):
    if not getattr(args, attr):
        print("Invalid argument, {} is not enabled".format(attr), file=sys.stderr)
        return False
    return True

def configure_generator(config, args) -> bool:
    if args.generator:
        config.generator = True

    if args.room_size:
        if not arg_valid("generator", args):
            return False
        config.generator_min_room_size = args.room_size[0]
        config.generator_max_room_size = args.room_size[1]

    if args.fill_rate:
        if not arg_valid("generator", args):
            return False
        config.generator_obstacle_fill_min = args.fill_rate[0]
        config.generator_obstacle_fill_max = args.fill_rate[1]

    if args.generator_type:
        if not arg_valid("generator", args):
            return False
        config.generator_gen_type = args.generator_gen_type

    if args.num_maps:
        if not arg_valid("generator", args):
            return False
        config.generator_nr_of_examples = args.num_maps

    return True

def configure_analyzer(config, args) -> bool:
    if args.analyzer:
        config.analyzer = True

    return True

def configure_trainer(config, args) -> bool:
    if args.trainer:
        config.trainer = True

    return True

def configure_visualiser(config, args) -> bool:
    if args.visualiser:
        config.load_simulator = True
        config.simulator_graphics = True

    if args.visualiser_flags is not None:
        if not arg_valid("visualiser", args):
            return False

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
                data += "win-origin 0 0\n"
                data += "undecorated true\n"
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

    return True

def configure_common(config, args) -> bool:
    # for generator & analyzer
    config.num_dim = args.dims

    config.simulator_write_debug_level = getattr(DebugLevel, args.debug)

    if args.list_algorithms:
        print("Available algorithms:")
        for key in AlgorithmManager.builtins.keys():
            print(f"  {key}")
        print("Or specify your own file with a class that inherits from Algorithm")
        return True

    if args.algorithms:
        algorithms = AlgorithmManager.load_all(args.algorithms)
        if not all(algorithms):
            invalid_algorithms = [args.algorithms[i] for i in range(len(algorithms)) if algorithms[i] is None]
            invalid_str = ",".join('"' + a + '"' for a in invalid_algorithms)
            valid_str = ",".join('"' + a + '"' for a in AlgorithmManager.builtins.keys())
            print(f"Invalid algorithm(s) specified: {invalid_str}")
            print(f"Available algorithms: {valid_str}")
            print("Or specify your own file with a class that inherits from Algorithm")
            return False
        config.algorithms = dict(algorithms)
    
    return True

def configure_and_run(args):
    config = Configuration()

    if not configure_common(config, args) or \
       not configure_generator(config, args) or \
       not configure_analyzer(config, args) or \
       not configure_trainer(config, args) or \
       not configure_visualiser(config, args):
        return False

    mr = MainRunner(config)
    mr.run()
    return True

def main() -> bool:
    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench runner",
                                     formatter_class=argparse.RawTextHelpFormatter)
    
    # Run arguments
    parser.add_argument("-v", "--visualiser", action='store_true', help="run visualiser (simulator with graphics)")
    parser.add_argument("-g", "--generator", action='store_true', help="run generator")
    parser.add_argument("-t", "--trainer", action='store_true', help="run trainer")
    parser.add_argument("-a", "--analyzer", action='store_true', help="run analyzer")

    # Visualiser arguments
    parser.add_argument("-V", dest="visualiser_flags", metavar="VISUALISER_FLAG", action='append',
                        help="[visualiser] options (overrides Panda3D's default Config.prc - see https://docs.panda3d.org/1.10/python/programming/configuration/configuring-panda3d#configuring-panda3d [windowed-fullscreen is an additional custom boolean flag])")

    # Generator arguments
    parser.add_argument("--room-size", nargs=2, type=int, help="[generator] min/max room size, in format \"min max\"")
    parser.add_argument("--fill-rate", nargs=2, type=float, help="[generator] min/max fill rate in random fill rooms")
    parser.add_argument("--generator-type", choices=list(Generator.AVAILABLE_GENERATION_METHODS), help="[generator] generator type")
    parser.add_argument("--num-maps", type=int, help="[generator] number of maps to generate")

    # Miscellaneous
    parser.add_argument("--dims", type=int, help="[generator|analyzer] number of dimensions", default=3)
    parser.add_argument("--algorithms", help="[visualiser|analyzer] algorithms to load (either built-in algorithm name or module file path)", nargs="+")
    parser.add_argument("--list-algorithms", action="store_true", help="[visualiser|analyzer] output list of available built-in algorithms")

    parser.add_argument("-d", "--debug", choices=['NONE', 'BASIC', 'LOW', 'MEDIUM', 'HIGH'], default='LOW', help="debug level when running, default is low")

    args = parser.parse_args()
    print("args:{}".format(args))
    return configure_and_run(args)


if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
