from typing import Any, List, Tuple
from collections.abc import MutableSequence, Iterable
from structures.tracked_container import TrackedContainer

""" A TrackedList class, used to keep track of elements, that are being added or removed from a 
    list data structure. TrackedList overrides the built-in MutableSequence methods, in order to account for 
    changes in the list, having a minimal effect on performance. Modified items are appended to the TrackedContainer's 
    list, taking into account if the element was removed or it was being added, calling elem_removed(elem) or
    elem_added(elem) respectively."""


class TrackedList(TrackedContainer, MutableSequence):
    _list: list

    def __init__(self, init: Iterable = []) -> None:
        TrackedContainer.__init__(self)
        self._list = [x for x in init]

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
