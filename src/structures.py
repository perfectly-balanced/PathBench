from typing import Tuple, NamedTuple
import torch, copy

class Point(torch.Tensor):
    """
    A point has a variable number of coordinates specified in the constructor.
    Each point is immutable.
    """
    pos: Tuple[int, ...]

    def __new__(cls, *ords, **kwargs):
        return torch.Tensor.__new__(cls, len(ords))

    def __init__(self, *ords, **kwargs):
        if ("x" in kwargs and "y" in kwargs):
            ords = (kwargs["x"], kwargs["y"])
            if ("z" in kwargs): ords = (*ords, kwargs["z"])
        assert (len(ords) > 1), "Needs at least two dimensions"
        super().__init__()
        self.pos = tuple(ords)
        self.data = torch.FloatTensor(self.pos)
    
    @property
    def x(self) -> int:
        return self.pos[0]

    @property
    def y(self) -> int:
        return self.pos[1]

    @property
    def z(self) -> int:
        assert len(self.pos) > 2, "Point has no Z-coordinate"
        return self.pos[2]
    
    def __getitem__(self, idx):
        return self.pos[idx]

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(c) for c in self.pos])

    @staticmethod
    def from_tensor(inp: torch.Tensor) -> 'Point':
        assert len(list(inp.size())) == 1
        return Point(*[int(torch.round(inp[i])) for i in range(list(inp.size())[0])])

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
        return f"Point({', '.join(str(i) for i in self.pos)})"

    def __copy__(self) -> 'Point':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: dict) -> 'Point':
        return Point(*self.pos)

class Size:
    """
    This tuple is used to describe the size of an object in n dimensions.
    The first three dimensions can be referred to as width, height, and depth, 
    which correspond to x, y, and z accordingly.
    """
    __size: Point

    def __init__(self, *sizes, **kwargs):
        if ("width" in kwargs and "height" in kwargs):
            sizes = (kwargs["width"], kwargs["height"])
            if "depth" in kwargs:
                sizes = (*sizes, kwargs["depth"])
        self.__size = Point(*sizes)

    @property
    def size(self):
        return self.__size
    
    @property
    def width(self) -> int:
        return self.__size.x
    
    @property
    def height(self) -> int:
        return self.__size.y
    
    @property
    def depth(self) -> int:
        return self.__size.z
    
    def __getitem__(self, idx) -> int:
        return self.__size[idx]
    
    def __len__(self) -> int:
        return len(self.__size)

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(i) for i in self.__size.pos])
    
    def __eq__(self, other: object) -> bool:
        return isinstance(other, Size) and self.__size == other.__size
    
    def __ne__(self, other: object) -> bool:
        return not (self == other)
    
    def __repr__(self) -> str:
        return f"Size({', '.join(str(i) for i in self.__size.pos)})"
