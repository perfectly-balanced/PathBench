from typing import Tuple, NamedTuple, Final, Optional, Callable
from numbers import Real, Integral
import torch
import copy

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

    @property
    def n_dim(self) -> int:
        return len(self.pos)
    
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
        return torch.Tensor([float(i) for i in self._size.pos])
    
    def __eq__(self, other: object) -> bool:
        return isinstance(other, Size) and self._size == other._size
    
    def __ne__(self, other: object) -> bool:
        return not (self == other)
    
    def __repr__(self) -> str:
        return f"Size({', '.join(str(i) for i in self._size.pos)})"


class Colour:
    """
    This tuple is used to describe an RGBA colour of an object.
    Alpha is optional and defaults to 1.0.
    If single argument 'x' given, this translates to (x, x, x, 1.0).
    """
    __data: Tuple[float, float, float, float]

    def __init__(self, *colours, **kwargs):
        keys = ("r", "g", "b", "a", "red", "green", "blue", "alpha")
        def get(kshort: str, klong: str, defaultable: bool = False) -> float:
            if kshort in kwargs:
                if klong in kwargs:
                    raise ValueError("invalid optional args, cannot have both '{}' & '{}' specified".format(kshort, klong))
                return kwargs[kshort]
            elif klong in kwargs:
                return kwargs[klong]
            elif defaultable:
                raise ValueError("missing optional arg '{}' / '{}'".format(kshort, klong))
            else:
                return None

        if len(colours) == 1 and len(kwargs) == 1 and ("a" in kwargs or "alpha" in kwargs):
            c = colours[0]
            self.__data = (c, c, c, get("a", "alpha"))
        elif len(kwargs) != 0 and any(k in kwargs for k in keys):            
            r = get("r", "red")
            g = get("g", "green")
            b = get("b", "blue")
            a = get("a", "alpha", True)

            assert len(kwargs) == (3 if a == None else 4), "unknown optional args"
            assert len(colours) == 0, "unknown positional args"

            self.__data = (r, g, b, a if a != None else 1.0)
        else:
            assert len(kwargs) == 0, "unknown optional args"
            if len(colours) == 1:
                c = colours[0]
                self.__data = (c, c, c, 1.0)
            elif len(colours) == 3:
                self.__data = (*colours, 1.0)
            else:
                assert len(colours) == 4, "invalid number of args"
                self.__data = colours

        assert self.__data[0] >= 0 and self.__data[0] <= 1
        assert self.__data[1] >= 0 and self.__data[1] <= 1
        assert self.__data[2] >= 0 and self.__data[2] <= 1
        assert self.__data[3] >= 0 and self.__data[3] <= 1

    @property
    def colours(self):
        return self.__data
    
    @property
    def red(self):
        return self.__data[0]

    @property
    def r(self):
        return self.red
    
    @property
    def green(self):
        return self.__data[1]

    @property
    def g(self):
        return self.green

    @property
    def blue(self):
        return self.__data[2]

    @property
    def b(self):
        return self.blue
    
    @property
    def alpha(self):
        return self.__data[3]

    @property
    def a(self):
        return self.alpha

    def with_red(self, value: float) -> 'Colour':
        return Colour(value, self.g, self.b, self.a)
    
    def with_r(self, value: float) -> 'Colour':
        return self.with_red(value)
    
    def with_green(self, value: float) -> 'Colour':
        return Colour(self.r, value, self.b, self.a)
    
    def with_g(self, value: float) -> 'Colour':
        return self.with_green(value)

    def with_blue(self, value: float) -> 'Colour':
        return Colour(self.r, self.g, value, self.a)
    
    def with_b(self, value: float) -> 'Colour':
        return self.with_blue(value)
    
    def with_alpha(self, value: float) -> 'Colour':
        return Colour(self.r, self.g, self.b, value)
    
    def with_a(self, value: float) -> 'Colour':
        return self.with_alpha(value)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Colour) and self.__data == other.__data
    
    def __ne__(self, other: object) -> bool:
        return not (self == other)
    
    def __repr__(self) -> str:
        return f"Colour({', '.join(str(i) for i in self.__data)})"
    
    def __getitem__(self, index):
        return self.__data[index]

WHITE: Final = Colour(1)
BLACK: Final = Colour(0)
RED: Final = Colour(1, 0, 0)
GREEN: Final = Colour(0, 1, 0)
BLUE: Final = Colour(0, 0, 1)
TRANSPARENT: Final = WHITE.with_alpha(0)

class DynamicColour:
    __colour: Colour
    __callback: Optional[Callable[['DynamicColour'], None]]
    __name: Optional[str]
    __visible: bool

    def __init__(self, colour: Colour, name: Optional[str] = None, callback: Optional[Callable[['DynamicColour'], None]] = None, visible: bool = True):
        self.__colour = colour
        self.__name = name
        self.__visible = visible
        self.__callback = callback
    
    @property
    def name(self) -> Optional[str]:
        return self.__name
    
    @property
    def deduced_colour(self) -> Colour:
        return self.__colour if self.visible else TRANSPARENT

    @property
    def colour(self) -> str:
        return 'colour'

    @colour.getter
    def colour(self) -> Colour:
        return self.__colour
    
    @colour.setter
    def colour(self, colour: Colour) -> None:
        self.__colour = colour
        if self.__callback is not None:
            self.__callback(self)
    
    @property
    def visible(self) -> str:
        return 'visible'

    @visible.getter
    def visible(self) -> bool:
        return self.__visible
    
    @visible.setter
    def visible(self, visible: bool) -> None:
        self.__visible = visible
        if self.__callback is not None:
            self.__callback(self)
    
    def __repr__(self) -> str:
        return f"DynamicColour(colour={self.__colour}, visible={self.visible})"

    def __call__(self) -> Colour:
        return self.deduced_colour