from typing import Any, List, Tuple
from collections.abc import MutableSequence

from structures.tracked import Tracked

class TrackedList(Tracked, MutableSequence):
    _list: list

    def __init__(self) -> None:
        Tracked.__init__(self)
        self._list = list()

    def __len__(self):
        return len(self._list)

    def __delitem__(self, index):
        self.modified.append(self._list[index])
        self._list.__delitem__(index)

    def insert(self, index, value):
        self.modified.append(value)
        self._list.insert(index, value)

    def __setitem__(self, index, value):
        self.modified.append(value)
        self.modified.append(self._list[index])
        self._list.__setitem__(index, value)

    def __getitem__(self, index):
        return self._list.__getitem__(index)

    def append(self, value):
        self.insert(len(self), value)

    def __repr__(self):
        return "TrackedList({0})".format(self._list)