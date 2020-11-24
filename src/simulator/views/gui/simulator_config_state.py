from simulator.services.persistent_state import PersistentStateObject
from structures import Point

from typing import Dict, Any

class SimulatorConfigState(PersistentStateObject):
    mp: str
    algo: str
    ani: str
    agent: Point
    goal: Point

    def _from_json(self, data: Dict[str, Any]) -> None:
        self.mp = data["mp"]
        self.algo = data["algo"]
        self.ani = data["ani"]
        self.agent = Point(*data["agent"])
        self.goal = Point(*data["goal"])

    def _to_json(self) -> Dict[str, Any]:
        data = {}
        data["mp"] = self.mp
        data["algo"] = self.algo
        data["ani"] = self.ani
        data["agent"] = tuple(self.agent)
        data["goal"] = tuple(self.goal)
        return data
