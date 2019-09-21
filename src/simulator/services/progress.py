from typing import Callable


class Progress:
    __steps: int
    __current_step: int
    __last_perc: int
    __handler: Callable[[int], None]

    def __init__(self, steps: int, handler: Callable[[int], None]) -> None:
        self.__steps = steps
        self.__handler = handler

    def start(self):
        self.__current_step = 0
        self.__last_perc = 0
        self.__handler(self.__last_perc)

    def step(self) -> None:
        self.__current_step += 1
        new_perc: int = int(float(self.__current_step) / float(self.__steps) * 100)

        if self.__last_perc != new_perc:
            self.__last_perc = new_perc
            self.__handler(self.__last_perc)
