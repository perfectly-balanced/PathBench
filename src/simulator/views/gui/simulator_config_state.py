from simulator.services.persistent_state import PersistentStateObject
from typing import Dict, Any

class SimulatorConfigState(PersistentStateObject):
    mp: str
    algo: str
    ani: str

    def _from_json(self, data: Dict[str, Any]) -> None:
        self.mp = data["mp"]
        self.algo = data["algo"]
        self.ani = data["ani"]

    def _to_json(self) -> Dict[str, Any]:
        data = {}
        data["mp"] = self.mp
        data["algo"] = self.algo
        data["ani"] = self.ani
        return data
