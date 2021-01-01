class Event:
    """
    A superclass for any events that might be generated by an
    object and sent to the EventManager.
    """
    _name: str
    _unifiable: bool

    def __init__(self, name: str = "Generic event", unifiable: bool = False) -> None:
        self._name = name
        self._unifiable = unifiable

    def __str__(self) -> str:
        return self._name

    def _absorb(self, other: 'Event') -> None:
        assert type(other) == type(self)
        assert self._unifiable
