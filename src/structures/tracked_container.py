from typing import List, Any
from structures.tracked import Tracked

class TrackedContainer(Tracked):
    modified: List[Any]
    elems_were_removed: bool
    __started_processing_size: int

    def __init__(self) -> None:
        self.modified = []
        self.elems_were_removed = False

    def _elem_added(self, elem) -> None:
        self.modified.append(elem)

    def _elem_removed(self, elem) -> None:
        self.elems_were_removed = True
        self.modified.append(elem)

    def clear_tracking_data(self) -> None:
        self.modified.clear()
        self.elems_were_removed = False
