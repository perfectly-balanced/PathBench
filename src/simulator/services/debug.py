from io import StringIO
import sys
from enum import Enum
from functools import wraps
from typing import Any, Callable, Dict, List

from simulator.services.service import Service
from utility.progress import Progress
from utility.timer import Timer

class DebugLevel(Enum):
    """
    The debug level tells how much information should be printed.

    NONE: No information
    BASIC: Only basic information
    LOW: Somewhat verbose
    MEDIUM: Quite verbose
    HIGH: All information
    """

    NONE: int = -1
    BASIC: int = 0
    LOW: int = 1
    MEDIUM: int = 2
    HIGH: int = 3


class Debug(Service):
    """
    Service class for printing messages into the console
    """

    def write(self, message: Any, level: DebugLevel = DebugLevel.BASIC, end: str = "\n", timestamp: bool = True,
              streams: List[StringIO] = None) -> None:
        if not streams:
            streams = []

        """
        Method used for writing to the console
        :param message: The message
        :param level: The debug level of information
        :param end: Terminator string
        :param timestamp: If a timestamp should be added
        """
        if self.should_debug(level):
            message = str(message)
            if timestamp:
                message = "[{}] - {}".format(Timer.get_current_timestamp(), message)
            print(message, end=end)

            for stream in streams:
                stream.write(message + end)

    @staticmethod
    def write_error(message: Any) -> None:
        """
        Method for displaying errors
        :param message: The error message
        """
        print("ERROR [{}] - {}".format(Timer.get_current_timestamp(), str(message)), file=sys.stderr)

    def should_debug(self, level: DebugLevel) -> bool:
        return self._services.settings.simulator_write_debug_level.value >= level.value

    def debug_func(self, debug_level: DebugLevel) -> Callable:
        def debug_func_decorator(func: Callable) -> Callable:
            @wraps(func)
            def wrapper(*args, **kwargs):
                self.write("Started: {}".format(func.__name__), debug_level)
                timer = Timer()
                res = func(*args, **kwargs)
                exec_time = timer.stop()
                self.write("Finished: {}, execution time: {} seconds".format(func.__name__, round(exec_time, 2)), debug_level)
                return res

            return wrapper

        return debug_func_decorator

    def progress_debug(self, steps: int, level: DebugLevel) -> Progress:
        return Progress(steps, lambda p: self._services.debug.write("Progress: {}%".format(p), level))

    @staticmethod
    def pretty_dic_str(dic: Dict) -> str:
        return "{\n\t" + \
               "\n\t".join("{}: {},".format(k, v) for k, v in dic.items()) + \
               "\n}"
