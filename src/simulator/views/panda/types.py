from typing import Tuple, TypeVar, Union
from numbers import Real, Integral

IntPoint3 = Tuple[Integral, Integral, Integral]  # XYZ

class RGBAColour:
    """
    This tuple is used to describe an RGBA colour of an object.
    Alpha is optional and defaults to 1.0.
    If single argument 'x' given, this translates to (x, x, x, 1.0).
    """
    __data: Tuple[float, float, float, float]

    def __init__(self, *colours, **kwargs):
        keys = ("r", "g", "b", "a", "red", "green", "blue", "alpha")
        if len(kwargs) != 0 and any(k in kwargs for k in keys):
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
    
    def __eq__(self, other: object) -> bool:
        return isinstance(other, Colour) and self.__data == other.__data
    
    def __ne__(self, other: object) -> bool:
        return not (self == other)
    
    def __repr__(self) -> str:
        return f"Colour({', '.join(str(i) for i in self.__data)})"
    
    def __getitem__(self, index):
        return self.__data[index]
