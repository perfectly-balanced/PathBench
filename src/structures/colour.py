from typing import Tuple, Callable

from utility.compatibility import Final

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

        assert self.__data[0] >= 0 and self.__data[0] <= 1, self.__data
        assert self.__data[1] >= 0 and self.__data[1] <= 1, self.__data
        assert self.__data[2] >= 0 and self.__data[2] <= 1, self.__data
        assert self.__data[3] >= 0 and self.__data[3] <= 1, self.__data

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

    def __apply_arithmetic_op(self, other: 'Colour', op: Callable[[float, float], float]) -> 'Colour':
        def clamp(x: float) -> float:
            return max(0, min(x, 1))

        r, g, b, a = self.__data
        ro, go, bo, ao = other.__data
        return Colour(clamp(op(r, ro)), clamp(op(g, go)), clamp(op(b, bo)), clamp(op(a, ao)))

    def __add__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, lambda a, b: (a + b))

    def __sub__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, lambda a, b: (a - b))

    def __mul__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, lambda a, b: (a * b))

    def __truediv__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, lambda a, b: (a / b))

    def __repr__(self) -> str:
        return f"Colour({', '.join(str(i) for i in self.__data)})"

    def __getitem__(self, index):
        return self.__data[index]

    def __hash__(self) -> int:
        return hash(self.__data)


WHITE: Final[Colour] = Colour(1)
BLACK: Final[Colour] = Colour(0)
RED: Final[Colour] = Colour(1, 0, 0)
GREEN: Final[Colour] = Colour(0, 1, 0)
BLUE: Final[Colour] = Colour(0, 0, 1)
TRANSPARENT: Final[Colour] = WHITE.with_alpha(0)
