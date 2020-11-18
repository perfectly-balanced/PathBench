from structures.tracked_container import TrackedContainer
from structures.heap import Heap

""" A TrackedHeap class, used to keep track of elements, that are being added or removed from a 
    heap data structure. TrackedHeap and heap share a common interface, in order to account for 
    changes, while having a minimal effect on performance. Modified items are appended to the 
    TrackedContainer's list, taking into account if the element was being removed or being added,
    calling elem_removed(elem) or elem_added(elem) respectively."""

class TrackedHeap(Heap, TrackedContainer):
    def __init__(self) -> None:
        Heap.__init__(self)
        TrackedContainer.__init__(self)

    def push(self, elem):
        self._elem_added(elem)
        super().push(elem)

    def pop(self):
        elem = super().pop()
        self._elem_removed(elem)
        return elem

    def __len__(self):
        return Heap.__len__(self)

    def __contains__(self, item):
        return Heap.__contains__(self, item)

    def __iter__(self):
        return Heap.__iter__(self)

    def __repr__(self):
        return "TrackedHeap({0})".format(self._list)
