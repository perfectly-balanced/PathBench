from heapq import heappush, heappop

class Heap():
    _list: list

    def __init__(self) -> None:
        self._list = list()

    def push(self, elem):
        heappush(self._list, elem)

    def pop(self):
        elem = heappop(self._list)
        return elem

    def __len__(self):
        return len(self._list)

    def __contains__(self, item):
        return item in self._list

    def __iter__(self):
        return iter(self._list)

    def __repr__(self):
        return "Heap({0})".format(self._list)
