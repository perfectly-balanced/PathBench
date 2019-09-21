import copy
from typing import Dict

from algorithms.configuration.entities.entity import Entity


class Agent(Entity):
    """
    Represents the main entity. It is currently the only entity that can be moved and it should be unique to the map.
    The agent's scope is to reach the goal.
    """

    def __str__(self) -> str:
        return "Agent: {position: " + str(self.position) + ", radius: " + str(self.radius) + "}"

    def __copy__(self) -> 'Agent':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Agent':
        return Agent(self.position, self.radius)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Agent):
            return False
        return super().__eq__(other)
