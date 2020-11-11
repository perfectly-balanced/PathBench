from abc import ABC, abstractmethod

class Tracked():
    @abstractmethod
    def clear_tracking_data(self) -> None:
        pass
