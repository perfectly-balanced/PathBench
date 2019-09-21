# -*- coding: utf-8 -*-

import copy
from typing import Dict

from structures import Point


class Entity:
    """
    This class is the main class for all objects found on a map.
    Every object from the map must extend this class.
    """
    __position: Point
    __radius: float

    def __init__(self, position: Point, radius: float = 0) -> None:
        """
        :param position: The position of the entity
        :param radius: The radius of the entity
        """
        self.__position = position
        self.__radius = radius

    @property
    def position(self) -> str:
        return "position"

    @position.getter
    def position(self) -> Point:
        """
        Position getter
        :return: The position of the entity
        """
        return self.__position

    @position.setter
    def position(self, value: Point) -> None:
        """
        Position setter
        :param value: The new value
        """
        self.__position = value

    @property
    def radius(self) -> str:
        return "radius"

    @radius.getter
    def radius(self) -> int:
        """
        Radius getter
        :return: The object radius as integer
        """
        return int(self.__radius)

    @radius.setter
    def radius(self, value: float) -> None:
        """
        Radius setter
        :param value: The radius value
        """
        self.__radius = value

    def __str__(self) -> str:
        return "Entity: {position: " + str(self.position) + ", radius: " + str(self.radius) + "}"

    def __copy__(self) -> 'Entity':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Entity':
        return Entity(self.position, self.radius)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Entity):
            return False
        return self.position == other.position and self.radius == other.radius

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)
