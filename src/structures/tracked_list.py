from typing import Any, List, Tuple
from collections.abc import MutableSequence

from structures.tracked_container import TrackedContainer

class TrackedList(TrackedContainer, MutableSequence):
    _list: list

    def __init__(self) -> None:
        TrackedContainer.__init__(self)
        self._list = list()

    def __len__(self):
        return len(self._list)

    def __delitem__(self, index):
        self._elem_removed(self._list[index])
        self._list.__delitem__(index)

    def insert(self, index, value):
        self._elem_added(value)
        self._list.insert(index, value)

    def __setitem__(self, index, value):
        self._elem_removed(self._list[index])
        self._elem_added(value)
        self._list.__setitem__(index, value)

    def __getitem__(self, index):
        return self._list.__getitem__(index)

    def append(self, value):
        self.insert(len(self), value)

    def __repr__(self):
        return "TrackedList({0})".format(self._list)