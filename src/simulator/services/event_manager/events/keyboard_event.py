from simulator.services.event_manager.events.event import Event


class KeyboardEvent(Event):
    """
    Keyboard or mouse input event.
    """
    key: int

    def __init__(self, key: int) -> None:
        super().__init__()
        self._name = "Keyboard event"
        self.key = key

    def __str__(self) -> str:
        return '%s, char=%s' % (self._name, self.key)
