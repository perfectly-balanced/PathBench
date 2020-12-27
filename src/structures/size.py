from typing import Tuple, Callable

import torch

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
        return self.values[0]

    @property
    def height(self) -> int:
        return self.values[1]

    @property
    def depth(self) -> int:
        return self.values[2]

    @property
    def n_dim(self) -> int:
        return len(self.values)

    def __len__(self) -> int:
        return self.n_dim

    def __getitem__(self, idx) -> int:
        return self.values[idx]

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(i) for i in self._size.values])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Size) and self.values == other.values

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __apply_arithmetic_op(self, other: 'Size', op: Callable[[int, int], int]) -> 'Size':
        assert type(other) is Size
        return Size(*(op(i, j) for i, j in zip(self.values, other.values)))

    def __add__(self, other: 'Size') -> 'Size':
        return self.__apply_arithmetic_op(other, lambda a, b: (a + b))

    def __sub__(self, other: 'Size') -> 'Size':
        return self.__apply_arithmetic_op(other, lambda a, b: (a - b))

    def __mul__(self, other: 'Size') -> 'Size':
        return self.__apply_arithmetic_op(other, lambda a, b: (a * b))

    def __floordiv__(self, other: 'Size') -> 'Size':
        return self.__apply_arithmetic_op(other, lambda a, b: (a // b))

    def __truediv__(self, other: 'Size') -> 'Size':
        return self.__floordiv__(other)

    def __repr__(self) -> str:
        return f"Size({', '.join(str(i) for i in self.values)})"
