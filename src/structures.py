from typing import Tuple, NamedTuple

import torch

class Point(torch.Tensor):
    """
    A point has a variable number of coordinates specified in the constructor. We assume the point has at least two coordinates
    """
    pos: Tuple[int, ...]

    @staticmethod
    def __new__(cls, *ords):
        return super().__new__(cls, len(ords))

    def __init__(self, *ords):
        assert len(ords) > 1, "Needs at least two dimensions"
        super().__init__()
        self.pos = tuple(ords)
    
    @property
    def x(self) -> int:
        return self.pos[0]

    @property
    def y(self) -> int:
        return self.pos[1]

    @property
    def z(self) -> int:
        assert len(self.pos) > 2, "Not three dimensional"
        return self.pos[2]

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(c) for c in self.pos])

    @staticmethod
    def from_tensor(inp: torch.Tensor) -> 'Point':
        assert len(list(inp.size())) == 1
        return Point([int(torch.round(inp[i])) for i in range(list(inp.size())[0])])
        #return Point(int(torch.round(inp[0])), int(torch.round(inp[1])))

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Point) and (self.pos == other.pos)

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __lt__(self, other: object) -> bool:
        assert isinstance(other, Point), "Must compare with another point"
        return self.pos < other.pos

    def __hash__(self) -> int:
        return hash(self.pos)

    def __repr__(self) -> str:
        return f"Point({', '.join(i for i in self.pos)})"

class Size(NamedTuple):
    """
    This tuple is used to describe the size of an object. It has 2 parameters width and height.
    """
    width: int
    height: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.width), float(self.height)])
