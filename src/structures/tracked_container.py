from typing import List, Any
from structures.tracked import Tracked

""" TrackedContainer keeps track of elements that have been modified. It is used for optimisation of graphics,
    in order to process a smaller set of points, rather than all the points in each frame. Every changed element
    is appended to the modified list and the corresponding flag for added/removed is set. The goal is to minimize
    the overhead of tracking all those changes by storing only those elements. At the end, all the tracking data 
    can be cleared."""

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
