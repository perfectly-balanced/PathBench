from panda3d.core import LVector3
from enum import IntEnum, unique, Enum

@unique
class Face(IntEnum):
    LEFT = 0
    RIGHT = 1
    BACK = 2
    FRONT = 3
    BOTTOM = 4
    TOP = 5

def normalise(*args):
    v = LVector3(*args)
    v.normalize()
    return v
