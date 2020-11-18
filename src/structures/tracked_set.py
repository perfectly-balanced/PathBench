from typing import Iterable, Any, List
from collections.abc import MutableSet
from structures.tracked_container import TrackedContainer

""" A TrackedSet class, used to keep track of elements, that are added or removed from a 
    set data structure. TrackedSet overrides the built-in MutableSet methods in order to account for changes
    in the set, having a minimal effect on performance. Modified items are appended to the TrackedContainer's list,
    taking into account if the element was removed or being added, calling elem_removed(elem) or
    elem_added(elem) respectively."""

class TrackedSet(TrackedContainer, MutableSet):
    _set: set

    def __init__(self) -> None:
        TrackedContainer.__init__(self)

    def __new__(cls, iterable=None):
        self_obj = super(TrackedSet, cls).__new__(TrackedSet)
        self_obj._set = set() if iterable is None else set(iterable)
        return self_obj

    def __contains__(self, item):
        return item in self._set

    def __len__(self):
        return len(self._set)

    def __iter__(self):
        return iter(self._set)

    def __repr__(self):
        return "TrackedSet({0})".format(list(self._set))

    def add(self, elem: Any) -> None:
        old_len = len(self)
        self._set.add(elem)
        new_len = len(self)
        if old_len != new_len:
            self._elem_added(elem)

    def clear(self) -> None:
        for elem in self._set:
            self._elem_removed(elem)
        self._set.clear()

    def difference_update(self, *s: Iterable[Any]) -> None:
        __intersect = self._set.intersection(s)
        for elem in __intersect:
            self._elem_removed(elem)
        self._set.difference_update(s)

    def discard(self, element: Any) -> None:
        old_len = len(self)
        self._set.discard(element)
        new_len = len(self)
        if old_len != new_len:
            self._elem_removed(element)

    def intersection_update(self, *s: Iterable[Any]) -> None:
        __difference = self._set.difference(s)
        for elem in __difference:
            self._elem_removed(elem)
        self._set.intersection_update(s)

    def pop(self) -> Any:
        elem = self._set.pop()
        self._elem_removed(elem)

    def remove(self, element: Any) -> None:
        old_len = len(self)
        self._set.remove(element)
        new_len = len(self)
        if old_len != new_len:
            self._elem_removed(element)

    def symmetric_difference_update(self, s: Iterable[Any]) -> None:
        sym_diff = self._set.symmetric_difference(s)
        for elem in self._set:
            if elem not in sym_diff:
                self._elem_removed(elem)
        for elem in s:
            if elem in sym_diff:
                self._elem_added(elem)
        self._set.symmetric_difference(s)

    def update(self, *s: Iterable[Any]) -> None:
        for elem in s:
            if elem not in self._set:
                self._elem_added(elem)
        self._set.update(s)
