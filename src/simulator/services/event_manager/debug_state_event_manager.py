from typing import List, Any, TYPE_CHECKING
from weakref import WeakKeyDictionary

from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event

if TYPE_CHECKING:
    from simulator.services.services import Services


class DebugStateEventManager:
    """
    We coordinate communication between the Model, View, and Controller.
    """
    __listeners: WeakKeyDictionary
    __services: 'Services'

    def __init__(self, __services: 'Services') -> None:
        self.__listeners = WeakKeyDictionary()
        self.__services = __services

    def register_listener(self, listener: Any) -> None:
        """
        Adds a listener to our spam list.
        It will receive Post()ed events through it's notify(event) call.
        :param listener: The listener
        """
        self.__listeners[listener] = 1

    def unregister_listener(self, listener: None) -> None:
        """
        Remove a listener from our spam list.
        This is implemented but hardly used.
        Our weak ref spam list will auto remove any listeners who stop existing.
        :param listener: The listener
        """
        if listener in self.__listeners.keys():
            del self.__listeners[listener]

    def post(self, event: Event) -> None:
        """
        Post a new event to the message queue.
        It will be broadcast to all listeners.
        :param event: The event to post
        """
        
        for listener in list(self.__listeners.keys()):
            listener.notify(event)
