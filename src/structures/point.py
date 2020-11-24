from typing import Union, Tuple

import torch
import copy

class Point(torch.Tensor):
    """
    A point has a variable number of coordinates specified in the constructor.
    Each point is immutable.
    """
    _pos: Union[Tuple[float, ...], Tuple[int, ...]]
    _is_float: bool

    def __new__(cls, *ords, **kwargs):
        return torch.Tensor.__new__(cls, len(ords))

    def __init__(self, *ords, **kwargs):
        if ("x" in kwargs and "y" in kwargs):
            ords = (kwargs["x"], kwargs["y"])
            if ("z" in kwargs):
                ords = (*ords, kwargs["z"])
        assert (len(ords) > 1), "Needs at least two dimensions"
        super().__init__()
        all_integers = all(map(lambda x: float(x).is_integer(), ords))
        self._is_float = not all_integers
        cast_type = int if all_integers else float
        self._pos = tuple(cast_type(i) for i in ords)
        self.data = torch.FloatTensor(self._pos)

    @property
    def x(self) -> Union[float, int]:
        return self._pos[0]

    @property
    def y(self) -> Union[float, int]:
        return self._pos[1]

    @property
    def z(self) -> Union[float, int]:
        assert len(self._pos) > 2, "Point has no Z-coordinate"
        return self._pos[2]

    @property
    def n_dim(self) -> int:
        return len(self._pos)

    @property
    def pos(self) -> Union[Tuple[float, ...], Tuple[int, ...]]:
        return self._pos

    @property
    def is_float(self) -> bool:
        return self._is_float

    def __getitem__(self, idx) -> Union[float, int]:
        return self._pos[idx]

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(c) for c in self._pos])

    @staticmethod
    def from_tensor(inp: torch.Tensor) -> 'Point':
        assert len(list(inp.size())) == 1
        return Point(*[int(torch.round(inp[i])) for i in range(list(inp.size())[0])])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Point) and (self._pos == other._pos)

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __lt__(self, other: object) -> bool:
        assert isinstance(other, Point), "Must compare with another point"
        return self._pos < other._pos

    def __hash__(self) -> int:
        return hash(self._pos)

    def __repr__(self) -> str:
        # want to print floats as ints for test, if doing so wouldn't change them
        return f"Point({', '.join(str(i) for i in self._pos)})"

    def __copy__(self) -> 'Point':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: dict) -> 'Point':
        return Point(*self._pos)
