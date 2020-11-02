from typing import List, Any

class Tracked():
    modified: List[Any]
    __started_processing_size: int

    def __init__(self) -> None:
        self.modified = []

    def clear_tracking_data(self) -> None:
        self.modified.clear()

