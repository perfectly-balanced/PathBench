import copy
from typing import Dict

from algorithms.configuration.entities.entity import Entity


class Obstacle(Entity):
    """
    This is an entity which blocks movement access for the agent.
    The agent is not allowed to step on it or collide with it.
    """

    def __str__(self) -> str:
        return "Obstacle: {position: " + str(self.position) + ", radius: " + str(self.radius) + "}"

    def __copy__(self) -> 'Obstacle':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Obstacle':
        return Obstacle(self.position, self.radius)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Obstacle):
            return False
        return super().__eq__(other)
