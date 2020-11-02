from typing import List, Any

class Tracked():
    modified: List[Any]

    def __init__(self) -> None:
        self.modified = []

    def clear_tracking_data(self) -> None:
        self.modified.clear()
