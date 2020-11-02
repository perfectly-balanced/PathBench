from heapq import heappush, heappop

from structures.tracked import Tracked

class TrackedHeap(Tracked):
    _list: list

    def __init__(self) -> None:
        Tracked.__init__(self)
        self._list = list()

    def push(self, element):
        self.modified.append(element)
        heappush(self._list, element)

    def pop(self):
        element = heappop(self._list)
        self.modified.append(element)
        return element

    def __len__(self):
        return len(self._list)
    
    def __contains__(self, item):
        return item in self._list

    def __iter__(self):
        return iter(self._list)

    def __repr__(self):
        return "TrackedHeap({0})".format(self._list)