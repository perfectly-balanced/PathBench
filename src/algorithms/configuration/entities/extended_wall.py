import copy
from typing import Dict

from algorithms.configuration.entities.obstacle import Obstacle


class ExtendedWall(Obstacle):
    """
    This object is an obstacle (marked with grey on the map) that represents
    the safe area around a real obstacle.
    """

    def __str__(self) -> str:
        return "ExtendedWall: {position: " + str(self.position) + ", radius: " + str(self.radius) + "}"

    def __copy__(self) -> 'ExtendedWall':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'ExtendedWall':
        return ExtendedWall(self.position, self.radius)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Obstacle):
            return False
        return super().__eq__(other)
