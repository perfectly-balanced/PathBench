from typing import Tuple, Callable

import torch
import numpy as np

class Size:
    """
    This tuple is used to describe the size of an object in n dimensions.
    The first three dimensions can be referred to as width, height, and depth, 
    which correspond to x, y, and z accordingly.
    """

    __values: Tuple[int, ...]

    def __init__(self, *values, **kwargs):
        if "width" in kwargs and "height" in kwargs:
            assert len(values) == 0
            values = (kwargs["width"], kwargs["height"])
            if "depth" in kwargs:
                values = (*values, kwargs["depth"])

        assert (len(values) > 1), "Needs at least two dimensions"
        assert all(map(lambda x: int(x) == x, values)), "Size elements must be of integral type"
        self.__values = values

    @property
    def values(self) -> Tuple[int, ...]:
        return self.__values

    @property
    def width(self) -> int:
        return self.__values[0]

    @property
    def height(self) -> int:
        return self.__values[1]

    @property
    def depth(self) -> int:
        return self.__values[2]

    @property
    def n_dim(self) -> int:
        return len(self.__values)

    def __len__(self) -> int:
        return self.n_dim

    def __getitem__(self, idx) -> int:
        return self.__values[idx]

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(i) for i in self.__values])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Size) and self.__values == other.__values

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __add__(self, other: 'Size') -> 'Size':
        return Size(*np.add(self.__values, other.__values))

    def __sub__(self, other: 'Size') -> 'Size':
        return Size(*np.subtract(self.__values, other.__values))

    def __mul__(self, other: 'Size') -> 'Size':
        return Size(*np.multiply(self.__values, other.__values))

    def __floordiv__(self, other: 'Size') -> 'Size':
        return Size(*np.floor_divide(self.__values, other.__values))

    def __truediv__(self, other: 'Size') -> 'Size':
        return self.__floordiv__(other)

    def __repr__(self) -> str:
        return f"Size({', '.join(str(i) for i in self.__values)})"
