import copy
from typing import Dict

from algorithms.configuration.entities.entity import Entity
from structures import Point


class Trace(Entity):
    """
    This entity marks the path chosen by the algorithms.
    """

    def __init__(self, position: Point) -> None:
        super().__init__(position, 1)

    def __repr__(self) -> str:
        return "Trace: {position: " + str(self.position) + "}"

    def __str__(self) -> str:
        return self.__repr__()

    def __copy__(self) -> 'Trace':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Trace':
        return Trace(self.position)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Trace):
            return False
        return super().__eq__(other)

    def __hash__(self) -> int:
        return hash(self.position)
