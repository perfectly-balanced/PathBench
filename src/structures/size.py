from structures.point import Point

import torch

class Size:
    """
    This tuple is used to describe the size of an object in n dimensions.
    The first three dimensions can be referred to as width, height, and depth, 
    which correspond to x, y, and z accordingly.
    """
    _size: Point

    def __init__(self, *sizes, **kwargs):
        if ("width" in kwargs and "height" in kwargs):
            sizes = (kwargs["width"], kwargs["height"])
            if "depth" in kwargs:
                sizes = (*sizes, kwargs["depth"])
        self._size = Point(*sizes)

    @property
    def size(self):
        return self._size

    @property
    def width(self) -> int:
        return self._size.x

    @property
    def height(self) -> int:
        return self._size.y

    @property
    def depth(self) -> int:
        return self._size.z

    @property
    def n_dim(self) -> int:
        return self._size.n_dim

    def __getitem__(self, idx) -> int:
        return self._size[idx]

    def __len__(self) -> int:
        return len(self._size)

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(i) for i in self._size.values])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Size) and self._size == other._size

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __repr__(self) -> str:
        return f"Size({', '.join(str(i) for i in self._size.values)})"
