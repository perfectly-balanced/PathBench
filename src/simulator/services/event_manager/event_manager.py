from typing import List, Any, TYPE_CHECKING
from weakref import WeakKeyDictionary

from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event

if TYPE_CHECKING:
    from simulator.services.services import Services


class EventManager:
    """
    We coordinate communication between the Model, View, and Controller.
    """
    __listeners: WeakKeyDictionary
    __tick_listeners: WeakKeyDictionary
    __event_queue: List[Event]
    __services: 'Services'

    def __init__(self, __services: 'Services') -> None:
        self.__listeners = WeakKeyDictionary()
        self.__tick_listeners = WeakKeyDictionary()
        self.__event_queue = []
        self.__services = __services

    def register_tick_listener(self, listener: Any) -> None:
        """
        Add a tick listener only
        :param listener: The listener
        """
        self.__tick_listeners[listener] = 1

    def unregister_tick_listener(self, listener: Any) -> None:
        """
        Remove the listener
        :param listener: The listener
        """
        if listener in self.__tick_listeners.keys():
            del self.__tick_listeners[listener]

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
    
    def broadcast(self, event: Event) -> None:
        """
        Broadcast an event to all listeners
        immediately.

        :param event: The event to post
        """
        
        for listener in list(self.__listeners.keys()):
            listener.notify(event)

    def post(self, event: Event) -> None:
        """
        Post a new event to the message queue.
        It will be broadcast to all listeners.
        :param event: The event to post
        """

        # unify event with an existing one if
        # allowed -> prevents spamming of events.
        if event._unifiable:
            for e in self.__event_queue:
                if type(e) == type(event):
                    e._absorb(event)
                    return

        self.__event_queue.append(event)
        self.__services.debug.write(str(event), DebugLevel.MEDIUM)

    def tick(self) -> None:
        """
        Sends all messages form the event queue
        """
        for tick_listener in self.__tick_listeners.keys():
            tick_listener.tick()
        
        # send all events
        while len(self.__event_queue) > 0:
            event = self.__event_queue.pop(0)
            self.broadcast(event)
