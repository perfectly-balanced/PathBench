import copy
from typing import Dict

from algorithms.configuration.entities.entity import Entity


class Goal(Entity):
    """
    This entity represents the exit point for the agent.
    """

    def __str__(self) -> str:
        return "Goal: {position: " + str(self.position) + ", radius: " + str(self.radius) + "}"

    def __copy__(self) -> 'Goal':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Goal':
        return Goal(self.position, self.radius)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Goal):
            return False
        return super().__eq__(other)
