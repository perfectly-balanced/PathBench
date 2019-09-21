from typing import Dict, List

from simulator.services.resources.directory import Directory
from simulator.services.services import Services


class Atlas(Directory):
    def __init__(self, services: Services, name: str, parent: str, create: bool = False) -> None:
        super().__init__(services, name, parent, create)

        if create:
            metadata: Dict[str, any] = {
                "next_index": 0,
            }
            self._save_metadata(metadata)

    def append(self, obj: any) -> None:
        self.save(str(self._get_next_index()), obj)
        self._increment_index()

    def load_all(self, max_els: int = float("inf")) -> List[any]:
        ret: List[any] = []
        idx: int = 0

        while idx < max_els:
            obj: any = self.load(str(idx))
            if obj:
                ret.append(obj)
                idx += 1
            else:
                break

        return ret

    def _get_next_index(self) -> int:
        metadata: Dict[str, any] = self._get_metadata()
        return metadata["next_index"]

    def _increment_index(self) -> None:
        metadata: Dict[str, any] = self._get_metadata()
        metadata["next_index"] += 1
        self._save_metadata(metadata)

    def _save_metadata(self, metadata: Dict[str, any]) -> None:
        super().save("metadata", metadata)

    def _get_metadata(self) -> Dict[str, any]:
        return super().load("metadata")
