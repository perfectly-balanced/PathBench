from abc import ABC, abstractmethod

class Tracked(ABC):
    @abstractmethod
    def clear_tracking_data(self) -> None:
        pass
