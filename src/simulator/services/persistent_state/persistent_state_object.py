from abc import ABC, abstractmethod
from typing import Dict, Any

class PersistentStateObject(ABC):
    _state: 'PersistentState'

    def __init__(self, state: 'PersistentState'):
        self._state = state

    @abstractmethod
    def _from_json(self, data: Dict[str, Any]) -> None:
        ...

    @abstractmethod
    def _to_json(self) -> Dict[str, Any]:
        ...
