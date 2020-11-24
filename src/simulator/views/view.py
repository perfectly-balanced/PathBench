from typing import List, Optional

from simulator.models.model import Model
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.initialise_event import InitialiseEvent
from simulator.services.event_manager.events.reinit_event import ReinitEvent
from simulator.services.services import Services


class View:
    _services: Services
    _model: Optional[Model]
    _initialised: bool
    _root_view: Optional['View']
    _children: List['View']

    def __init__(self, services: Services, model: Optional[Model], root_view: Optional['View']) -> None:
        """
        ev_manager (EventManager): Allows posting messages to the event queue.
        model (GameEngine): a strong reference to the game Model.

        Attributes:
        initialised (bool): self.services.render_engine is ready to draw.
        screen (self.services.render_engine.Surface): the screen surface.
        clock (self.services.render_engine.time.Clock): keeps the fps constant.
        """
        self._services = services
        self._services.ev_manager.register_listener(self)
        self._model = model
        self._initialised = False
        self._root_view = root_view
        self._children = []

        if self._root_view is not None:
            self._root_view.add_child(self)

    def add_child(self, view: 'View') -> None:
        self._children.append(view)
        view._root_view = self

    def remove_child(self, view: 'View') -> None:
        self._children.remove(view)
        view._root_view = None

    def __del__(self) -> None:
        if self._root_view is not None:
            self._root_view.remove_child(self)

    def notify(self, event: Event) -> None:
        if isinstance(event, InitialiseEvent):
            self.initialise()
            self._initialised = True
        elif isinstance(event, ReinitEvent):
            from simulator.views.main_view import MainView
            if not isinstance(self, MainView):
                self._services.ev_manager.unregister_listener(self)
                if self._root_view is not None:
                    self._root_view.remove_child(self)

    def initialise(self) -> None:
        pass

    def update(self) -> None:
        pass
