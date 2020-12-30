from typing import Any, List, Tuple

import numpy as np

from structures.tracked import Tracked

class _TrackedGridView():
    _partial_index: Tuple[int, ...]
    _grid: 'TrackedGrid'
    _values: np.ndarray

    def __init__(self, grid: 'TrackedGrid', partial_index: Tuple[int, ...], view: np.ndarray) -> None:
        self._partial_index = partial_index
        self._grid = grid
        self._values = view

    @property
    def shape(self) -> Tuple[int, ...]:
        return self._values.shape

    @property
    def dtype(self) -> Any:
        return self._values.dtype

    @property
    def ndim(self) -> int:
        return self._values.ndim

    def __setitem__(self, index, value):
        if type(index) is tuple:
            self._grid.modified.append(((*self._partial_index, *index), self._values[index]))
            self._values[index] = value
        elif self._values.ndim == 1:
            self._grid.modified.append(((*self._partial_index, index), self._values[index]))
            self._values[index] = value
        else:
            return _TrackedGridView(self._grid, (*self._partial_index, index), self._values[index])

    def __getitem__(self, index):
        if type(index) is tuple or self._values.ndim == 1:
            return self._values[index]
        else:
            return _TrackedGridView(self._grid, (*self._partial_index, index), self._values[index])

class TrackedGrid(Tracked):
    modified: List[Tuple[Tuple[int, ...], Any]]  # (index, old value)
    _values: np.ndarray

    def __init__(self, *args, **kwargs) -> None:
        self._values = np.array(*args, **kwargs)
        self.modified = []

    @property
    def shape(self) -> Tuple[int, ...]:
        return self._values.shape

    @property
    def dtype(self) -> Any:
        return self._values.dtype

    @property
    def ndim(self) -> int:
        return self._values.ndim

    def insert(self, index, value):
        self._elem_added(value)
        self._list.insert(index, value)

    def __setitem__(self, index, value):
        if type(index) is tuple:
            self.modified.append((index, self._values[index]))
            self._values[index] = value
        else:
            return _TrackedGridView(self, (index,), self._values[index])

    def __getitem__(self, index):
        if type(index) is tuple:
            return self._values[index]
        else:
            return _TrackedGridView(self, (index,), self._values[index])

    def __repr__(self):
        return "TrackedGrid({0})".format(self._values)

    def clear_tracking_data(self) -> None:
        self.modified.clear()
