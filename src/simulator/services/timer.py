import time
import datetime
from datetime import datetime
from typing import Optional


class Timer:
    """
    Used to time executions
    """

    __last_pause_time: Optional[datetime]
    __pause_total_time: float
    __start_time: datetime

    def __init__(self) -> None:
        self.__start_time = None
        self.__last_pause_time = None
        self.__pause_total_time = 0
        self.start()

    def start(self) -> 'Timer':
        """
        Start the timer
        :return: Itself
        """
        self.__start_time = datetime.now()
        self.__pause_total_time = 0
        self.__last_pause_time = None
        return self

    def stop(self) -> float:
        """
        Report the delta time (it just reports the results does not actually stop)
        :return: The delta time
        """
        self.resume()
        return (datetime.now() - self.__start_time).total_seconds() - self.__pause_total_time

    def resume(self) -> None:
        """
        Resume the timer
        """
        if self.__last_pause_time is None:
            return
        self.__pause_total_time += (datetime.now() - self.__last_pause_time).total_seconds()
        self.__last_pause_time = None

    def pause(self) -> None:
        """
        Pause the timer
        """
        self.__last_pause_time = datetime.now()

    @staticmethod
    def get_current_timestamp():
        return datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S')
