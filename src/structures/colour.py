from typing import Tuple, Callable, Any
import numpy as np

from utility.compatibility import Final

class Colour:
    """
    This tuple is used to describe an RGBA colour of an object.
    Alpha is optional and defaults to 1.0.
    If single argument 'x' given, this translates to (x, x, x, 1.0).
    """
    __values: Tuple[float, float, float, float]

    def __init__(self, *colours, **kwargs):
        if len(colours) == 4:
            self.__values = colours
            assert len(kwargs) == 0, "unknown optional args"
            assert self.__values[0] >= 0 and self.__values[0] <= 1, self.__values
            assert self.__values[1] >= 0 and self.__values[1] <= 1, self.__values
            assert self.__values[2] >= 0 and self.__values[2] <= 1, self.__values
            assert self.__values[3] >= 0 and self.__values[3] <= 1, self.__values
            return

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
            self.__values = (c, c, c, get("a", "alpha"))
        elif len(kwargs) != 0 and any(k in kwargs for k in keys):
            r = get("r", "red")
            g = get("g", "green")
            b = get("b", "blue")
            a = get("a", "alpha", True)

            assert len(kwargs) == (3 if a == None else 4), "unknown optional args"
            assert len(colours) == 0, "unknown positional args"

            self.__values = (r, g, b, a if a != None else 1.0)
        else:
            assert len(kwargs) == 0, "unknown optional args"
            if len(colours) == 1:
                c = colours[0]
                self.__values = (c, c, c, 1.0)
            elif len(colours) == 3:
                self.__values = (*colours, 1.0)

        assert self.__values[0] >= 0 and self.__values[0] <= 1, self.__values
        assert self.__values[1] >= 0 and self.__values[1] <= 1, self.__values
        assert self.__values[2] >= 0 and self.__values[2] <= 1, self.__values
        assert self.__values[3] >= 0 and self.__values[3] <= 1, self.__values

    @property
    def values(self):
        return self.__values

    @property
    def red(self):
        return self.__values[0]

    @property
    def r(self):
        return self.red

    @property
    def green(self):
        return self.__values[1]

    @property
    def g(self):
        return self.green

    @property
    def blue(self):
        return self.__values[2]

    @property
    def b(self):
        return self.blue

    @property
    def alpha(self):
        return self.__values[3]

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
        return isinstance(other, Colour) and self.__values == other.__values

    def __ne__(self, other: object) -> bool:
        return not (self == other)

    def __apply_arithmetic_op(self, other: 'Colour', op: Callable[[Any, Any], Any]) -> 'Colour':
        return Colour(*np.clip(op(self.__values, other.__values), 0, 1))

    def __add__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, np.add)

    def __sub__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, np.subtract)

    def __mul__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, np.multiply)

    def __truediv__(self, other: 'Colour') -> 'Colour':
        return self.__apply_arithmetic_op(other, np.divide)

    def __repr__(self) -> str:
        return f"Colour({', '.join(str(i) for i in self.__values)})"

    def __getitem__(self, index):
        return self.__values[index]

    def __hash__(self) -> int:
        return hash(self.__values)


WHITE: Final[Colour] = Colour(1)
BLACK: Final[Colour] = Colour(0)
RED: Final[Colour] = Colour(1, 0, 0)
GREEN: Final[Colour] = Colour(0, 1, 0)
BLUE: Final[Colour] = Colour(0, 0, 1)
TRANSPARENT: Final[Colour] = WHITE.with_alpha(0)
