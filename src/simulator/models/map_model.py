import time

from utility.threading import Thread, Condition, cond_var_wait_for
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.sparse_map import SparseMap
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.reset_event import ResetEvent
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.services import Services
from simulator.services.timer import Timer
from structures import Point

class AlgorithmTerminated(Exception):
    pass

class MapModel(Model):
    speed: int
    key_frame_is_paused: bool
    last_thread: Thread
    frame_timer: Timer
    condition: Condition
    processing_key_frame: bool

    def __init__(self, services: Services) -> None:
        super().__init__(services)
        self._services.ev_manager.register_tick_listener(self)
        self.last_thread = None
        self.key_frame_is_paused = False
        self.cv = Condition()
        self.processing_key_frame = False
        self.frame_timer = Timer()
        self.speed = 1 if self._services.settings.simulator_grid_display else 20

        self._services.algorithm.set_root()

    def move_up(self) -> None:
        self.move(Point(self._services.algorithm.map.agent.position.x,
                        self._services.algorithm.map.agent.position.y - self.speed))

    def move_down(self) -> None:
        self.move(Point(self._services.algorithm.map.agent.position.x,
                        self._services.algorithm.map.agent.position.y + self.speed))

    def move_left(self):
        self.move(Point(self._services.algorithm.map.agent.position.x - self.speed,
                        self._services.algorithm.map.agent.position.y))

    def move_right(self) -> None:
        self.move(Point(self._services.algorithm.map.agent.position.x + self.speed,
                        self._services.algorithm.map.agent.position.y))

    def reset(self) -> None:
        self.stop_algorithm()
        self._services.algorithm.reset_algorithm()
        if self.processing_key_frame:
            self.processing_key_frame = False
            with self.cv:
                self.requires_key_frame = False
                self.cv.notify_all()
        with self.cv:
            cond_var_wait_for(self.cv, lambda: self.last_thread is None or self.requires_key_frame, timeout=0.8)
            if self.last_thread is not None:
                self.requires_key_frame = False
                self.cv.notify_all()
                cond_var_wait_for(self.cv, lambda: self.last_thread is None, timeout=0.8)
            self.requires_key_frame = False
            if self.last_thread is None:
                self._services.ev_manager.post(KeyFrameEvent(refresh=True))

    def move(self, to: Point) -> None:
        self.reset()
        self._services.algorithm.map.move_agent(to, True)

    def move_goal(self, to: Point) -> None:
        self.reset()
        self._services.algorithm.map.move(self._services.algorithm.map.goal, to, True)

    def stop_algorithm(self) -> None:
        self.key_frame_is_paused = True

    def resume_algorithm(self) -> None:
        self.key_frame_is_paused = False

    def toggle_pause_algorithm(self) -> None:
        self.key_frame_is_paused = not self.key_frame_is_paused

    @property
    def requires_key_frame(self) -> bool:
        return self._services.algorithm.instance.testing is not None and self._services.algorithm.instance.testing.requires_key_frame

    @requires_key_frame.setter
    def requires_key_frame(self, value) -> None:
        if self._services.algorithm.instance.testing is not None:
            self._services.algorithm.instance.testing.requires_key_frame = value

    def tick(self) -> None:
        if self.processing_key_frame:
            self.processing_key_frame = False
            with self.cv:
                self.requires_key_frame = False
                self.cv.notify_all()

        if not self.key_frame_is_paused and self.last_thread is not None:
            MAX_FRAME_DT = 1 / 16
            dt = self.frame_timer.stop()
            if self.requires_key_frame or (dt < MAX_FRAME_DT):
                with self.cv:
                    if cond_var_wait_for(self.cv, lambda: self.requires_key_frame or self.last_thread is None, timeout=(MAX_FRAME_DT - dt)):
                        self.processing_key_frame = True
                        self._services.ev_manager.post(KeyFrameEvent())
                self.frame_timer = Timer()

    def compute_trace(self) -> None:
        if self.last_thread:
            return

        self.reset()

        def compute_wrapper() -> None:
            self.key_frame_is_paused = False
            if self._services.settings.simulator_key_frame_speed > 0:
                self._services.algorithm.instance.set_condition(self.cv)
            self._services.ev_manager.post(KeyFrameEvent(refresh=True))
            try:
                self._services.algorithm.instance.find_path()
            except AlgorithmTerminated:
                self._services.debug.write("Terminated algorithm", DebugLevel.BASIC)
                self._services.ev_manager.post(KeyFrameEvent(refresh=True))
            if self._services.settings.simulator_key_frame_speed == 0:
                # no animation hence there hasn't been a chance to render
                # the last state of the algorithm.
                self._services.ev_manager.post(KeyFrameEvent(refresh=True))
            with self.cv:
                self.last_thread = None
                self.cv.notify()

        self.last_thread = Thread(target=compute_wrapper, daemon=True)
        self.last_thread.start()

    def toggle_convert_map(self) -> None:
        if isinstance(self._services.algorithm.map, DenseMap):
            self._services.debug.write("Converting map to SparseMap", DebugLevel.BASIC)
            mp = self._services.algorithm.map.convert_to_sparse_map()
        elif isinstance(self._services.algorithm.map, SparseMap):
            self._services.debug.write("Converting map to DenseMap", DebugLevel.BASIC)
            mp = self._services.algorithm.map.convert_to_dense_map()
        else:
            self._services.debug.write("Map conversion not applicable", DebugLevel.BASIC)
            return

        if mp is None:
            self._services.debug.write("Map conversion not applicable", DebugLevel.BASIC)
            return

        self.reset()
        timer: Timer = Timer()
        self._services.algorithm.map = mp
        self._services.debug.write("Done converting. Total time: " + str(timer.stop()), DebugLevel.BASIC)
        self._services.debug.write(self._services.algorithm.map, DebugLevel.MEDIUM)
        self._services.ev_manager.post(KeyFrameEvent())

    def notify(self, event: Event) -> None:
        """
        Called by an event in the message queue.
        """

        if isinstance(event, ResetEvent):
            self.reset()
