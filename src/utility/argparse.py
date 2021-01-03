import argparse
import random
from typing import List, Callable

from panda3d.core import load_prc_file_data
from screeninfo import get_monitors
import numpy as np
import torch
import re
import sys

from algorithms.configuration.configuration import Configuration
from algorithms.algorithm_manager import AlgorithmManager
from simulator.services.debug import DebugLevel
from utility.misc import flatten

def add_configuration_flags(parser: argparse.ArgumentParser,
                            help_prefix: str = "",
                            visualiser_flags: bool = False,
                            debug_flag: bool = False,
                            deterministic_flags: bool = False,
                            algorithms_flags: bool = False,
                            multiple_algorithms_specifiable: bool = True) -> Callable[[Configuration, argparse.Namespace], bool]:
    fs: List[Callable[[Configuration, argparse.Namespace], bool]] = []

    if visualiser_flags:
        parser.add_argument("-V", dest="visualiser_flags", metavar="VISUALISER_FLAG", action='append',
                            help=help_prefix + "window options (overrides Panda3D's default Config.prc - see https://docs.panda3d.org/1.10/python/programming/configuration/configuring-panda3d#configuring-panda3d [windowed-fullscreen is an additional custom boolean flag])")
        fs.append(configure_visualiser_flags)

    if debug_flag:
        parser.add_argument("-d", "--debug", choices=['NONE', 'BASIC', 'LOW', 'MEDIUM', 'HIGH'], default='LOW', help=help_prefix + "debug level when running, default is low")
        fs.append(configure_debug_flag)

    if deterministic_flags:
        parser.add_argument("--deterministic", action='store_true', help=help_prefix + "use pre-defined random seeds for deterministic exeuction")
        parser.add_argument("--std-random-seed", type=int, default=0, help=help_prefix + "'random' module random number generator seed")
        parser.add_argument("--numpy-random-seed", type=int, default=0, help=help_prefix + "'numpy' module random number generator seed")
        parser.add_argument("--torch-random-seed", type=int, default=0, help=help_prefix + "'torch' module random number generator seed")
        fs.append(configure_deterministic_flags)

    if algorithms_flags:
        if multiple_algorithms_specifiable:
            parser.add_argument("--algorithms", help=help_prefix + "algorithms to load (either built-in algorithm name or module file path)", nargs="+")
        else:
            parser.add_argument("--algorithm", help=help_prefix + "algorithm to load (either built-in algorithm name or module file path)")
        parser.add_argument("--list-algorithms", action="store_true", help=help_prefix + "output list of available built-in algorithms")
        fs.append(configure_algorithms_flags)

    def handler(config: Configuration, args: argparse.Namespace) -> bool:
        nonlocal fs
        for f in fs:
            if not f(config, args):
                return False
        return True

    return handler

def configure_visualiser_flags(config: Configuration, args: argparse.Namespace) -> bool:
    if args.visualiser_flags is not None:
        integral_pair_re = re.compile("\s*\(\s*[0-9]+\s*,\s*[0-9]+\s*\)\s*")
        naked_integral_pair_re = re.compile("\s*[0-9]+\s*[0-9]+\s*")

        data = ""

        def use_display_resolution():
            nonlocal data

            for m in get_monitors():
                data += "win-size {} {}\n".format(m.width, m.height)
                break  # only need to specify for first window

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

def configure_debug_flag(config: Configuration, args: argparse.Namespace) -> bool:
    config.simulator_write_debug_level = getattr(DebugLevel, args.debug)

    return True

def configure_deterministic_flags(config: Configuration, args: argparse.Namespace) -> bool:
    if args.deterministic:
        random.seed(args.std_random_seed)
        torch.manual_seed(args.torch_random_seed)
        np.random.seed(args.numpy_random_seed)

    return True

def configure_algorithms_flags(config: Configuration, args: argparse.Namespace) -> bool:
    if args.list_algorithms:
        print("Available algorithms:")
        for key in AlgorithmManager.builtins.keys():
            print(f"  {key}")
        print("Or specify your own file that contains a class that inherits from Algorithm")
        sys.exit(0)

    d = vars(args)
    if "algorithm" in d and d["algorithm"]:
        in_algorithms = [d["algorithm"]]
    elif "algorithms" in d:
        in_algorithms = d["algorithms"]
    else:
        in_algorithms = []

    if in_algorithms:
        print(in_algorithms)
        algorithms = AlgorithmManager.load_all(in_algorithms)
        if not all(algorithms):
            invalid_algorithms = [in_algorithms[i] for i in range(len(algorithms)) if not algorithms[i]]
            invalid_str = ",".join('"' + a + '"' for a in invalid_algorithms)
            valid_str = ",".join('"' + a + '"' for a in AlgorithmManager.builtins.keys())
            print(f"Invalid algorithm(s) specified: {invalid_str}", file=sys.stderr)
            print(f"Available algorithms: {valid_str}", file=sys.stderr)
            print("Or specify your own file that contains a class that inherits from Algorithm", file=sys.stderr)
            return False

        algorithms = list(flatten(algorithms, depth=1))

        # name uniqueness
        names = [a[0] for a in algorithms]
        if len(set(names)) != len(names):
            print("Name conflict detected in custom algorithm list:", names, file=sys.stderr)
            return False

        algorithms = dict(algorithms)
        config.algorithms = algorithms

    return True
