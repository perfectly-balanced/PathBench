from typing import NamedTuple

import torch


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


class Size(NamedTuple):
    """
    This tuple is used to describe the size of an object. It has 2 parameters width and height.
    """
    width: int
    height: int

    def to_tensor(self) -> torch.Tensor:
        return torch.Tensor([float(self.width), float(self.height)])
