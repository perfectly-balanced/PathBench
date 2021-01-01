from simulator.models.model import Model
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.initialise_event import InitialiseEvent
from simulator.services.event_manager.events.quit_event import QuitEvent
from simulator.services.services import Services


class MainModel(Model):
    """
    Tracks the game state.
    """
    __running: bool

    def __init__(self, services: Services) -> None:
        """
        evManager (EventManager): Allows posting messages to the event queue.

        Attributes:
        running (bool): True while the engine is online. Changed via QuitEvent().
        """

        super().__init__(services)
        self.__running = False

    def notify(self, event: Event) -> None:
        """
        Called by an event in the message queue.
        """

        if isinstance(event, QuitEvent):
            self.__running = False

    def run(self) -> None:
        """
        Starts the game engine loop.

        This pumps a Tick event into the message queue for each loop.
        The loop ends when this object hears a QuitEvent in notify().
        """
        self.__running = True
        self._services.ev_manager.post(InitialiseEvent())

        self.__run_main_loop()

    def __run_main_loop(self) -> None:
        while self.__running:
            self._services.ev_manager.tick()
