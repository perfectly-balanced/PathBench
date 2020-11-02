from typing import Iterable, Any, List
from collections.abc import MutableSet

from structures.tracked import Tracked

class TrackedSet(Tracked, MutableSet):
    _set: set

    def __init__(self) -> None:
        Tracked.__init__(self)

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

    def add(self, element: Any) -> None:
        old_len = len(self)
        self._set.add(element)
        new_len = len(self)
        if old_len != new_len:
            self.modified.append(element)

    def clear(self) -> None:
        raise NotImplementedError

    def difference_update(self, *s: Iterable[Any]) -> None:
        raise NotImplementedError

    def discard(self, element: Any) -> None:
        raise NotImplementedError

    def intersection_update(self, *s: Iterable[Any]) -> None:
        raise NotImplementedError

    def pop(self) -> Any:
        raise NotImplementedError

    def remove(self, element: Any) -> None:
        raise NotImplementedError

    def symmetric_difference_update(self, s: Iterable[Any]) -> None:
        raise NotImplementedError

    def update(self, *s: Iterable[Any]) -> None:
        raise NotImplementedError
