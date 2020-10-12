from typing import NamedTuple

import torch

class Point3D(NamedTuple, torch.Tensor):
    """
    A point in 3D space has 3 coordinates x, y, and z.
    """

    x: int
    y: int
    z: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.x), float(self.y), float(self.z)])

    @staticmethod
    def from_tensor(inp: torch.Tensor) -> 'Point3D':
        return Point3D(int(torch.round(inp[0])), int(torch.round(inp[1]), int(torch.round(inp[2]))))


class Point(NamedTuple, torch.Tensor):
    """
    A point has 2 coordinates x and y.
    """
    x: int
    y: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.x), float(self.y)])

    @staticmethod
    def from_tensor(inp: torch.Tensor) -> 'Point':
        return Point(int(torch.round(inp[0])), int(torch.round(inp[1])))


class Size3D(NamedTuple):
    """
    This tuple represents the size of objects in 3D. It has 3 parameters: length, width, and height.
    """
    length: int
    width: int
    height: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.length), float(self.width), float(self.height)])


class Size(NamedTuple):
    """
    This tuple is used to describe the size of an object. It has 2 parameters width and height.
    """
    width: int
    height: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.width), float(self.height)])
