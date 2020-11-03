from heapq import heappush, heappop

from structures.tracked import Tracked

class TrackedHeap(Tracked):
    _list: list

    def __init__(self) -> None:
        Tracked.__init__(self)
        self._list = list()

    def push(self, elem):
        self._added_elem(elem)
        heappush(self._list, elem)

    def pop(self):
        elem = heappop(self._list)
        self._removed_elem(elem)
        return elem

    def __len__(self):
        return len(self._list)
    
    def __contains__(self, item):
        return item in self._list

    def __iter__(self):
        return iter(self._list)

    def __repr__(self):
        return "TrackedHeap({0})".format(self._list)