import pygame
from threading import Thread, Condition

from algorithms.configuration.maps.sparse_map import SparseMap
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.services import Services
from simulator.services.timer import Timer
from structures import Point


class Map(Model):
    speed: int
    key_frame_is_paused: bool
    key_frame_timer: Timer
    last_thread: Thread
    key_frame_condition: Condition

    def __init__(self, services: Services) -> None:
        super().__init__(services)
        self._services.ev_manager.register_tick_listener(self)
        self.last_thread = None
        self.key_frame_timer = Timer()
        self.key_frame_is_paused = False
        self.key_frame_condition = None
        self.speed = 1 if self._services.settings.simulator_grid_display else 20

        self._services.algorithm.set_root()

    def move_up(self) -> None:
        self.reset()
        self.move(Point(self._services.algorithm.map.agent.position.x,
                        self._services.algorithm.map.agent.position.y - self.speed))

    def move_down(self) -> None:
        self.reset()
        self.move(Point(self._services.algorithm.map.agent.position.x,
                        self._services.algorithm.map.agent.position.y + self.speed))

    def move_left(self):
        self.reset()
        self.move(Point(self._services.algorithm.map.agent.position.x - self.speed,
                        self._services.algorithm.map.agent.position.y))

    def move_right(self) -> None:
        self.reset()
        self.move(Point(self._services.algorithm.map.agent.position.x + self.speed,
                        self._services.algorithm.map.agent.position.y))

    def reset(self) -> None:
        self.stop_algorithm()
        self._services.algorithm.reset_algorithm()

    def move(self, to: Point) -> None:
        self.reset()
        self._services.algorithm.map.move_agent(to, True)
        self._services.ev_manager.post(KeyFrameEvent())

    def move_goal(self, to: Point) -> None:
        self.reset()
        self._services.algorithm.map.move(self._services.algorithm.map.goal, to, True)
        self._services.ev_manager.post(KeyFrameEvent())

    def stop_algorithm(self) -> None:
        self.key_frame_is_paused = True

    def resume_algorithm(self) -> None:
        self.key_frame_is_paused = False

    def toggle_pause_algorithm(self) -> None:
        self.key_frame_is_paused = not self.key_frame_is_paused

    def tick(self) -> None:
        if self._services.settings.simulator_key_frame_speed > 0 and self.key_frame_condition is not None and \
                not self.key_frame_is_paused:
            if self.key_frame_timer.stop() > self._services.settings.simulator_key_frame_speed:
                self.key_frame_condition.acquire()
                self.key_frame_condition.notify()
                self.key_frame_condition.release()
                self._services.ev_manager.post(KeyFrameEvent())
                self.key_frame_timer = Timer()

    def compute_trace(self) -> None:
        if self.last_thread:
            return

        self.reset()

        def compute_wrapper() -> None:
            self.key_frame_is_paused = False
            if self._services.settings.simulator_key_frame_speed > 0:
                self._services.algorithm.instance.set_condition(self.key_frame_condition)
            self._services.algorithm.instance.find_path()
            self._services.ev_manager.post(KeyFrameEvent(is_first=True))
            self.key_frame_condition = None
            self.last_thread = None

        self.last_thread = Thread(target=compute_wrapper, daemon=True)
        self.key_frame_condition = Condition()
        self.last_thread.start()

    def toggle_convert_map(self) -> None:
        self.reset()
        from algorithms.configuration.maps.dense_map import DenseMap
        timer: Timer = Timer()
        if isinstance(self._services.algorithm.map, DenseMap):
            self._services.debug.write("Converting map to SparseMap", DebugLevel.BASIC)
            self._services.algorithm.map = self._services.algorithm.map.convert_to_sparse_map()
        elif isinstance(self._services.algorithm.map, SparseMap):
            self._services.debug.write("Converting map to DenseMap", DebugLevel.BASIC)
            self._services.algorithm.map = self._services.algorithm.map.convert_to_dense_map()
        self._services.debug.write("Done converting. Total time: " + str(timer.stop()), DebugLevel.BASIC)
        self._services.debug.write(self._services.algorithm.map, DebugLevel.MEDIUM)
        self._services.ev_manager.post(KeyFrameEvent())
