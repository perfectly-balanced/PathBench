from typing import List, Any

class Tracked():
    modified: List[Any]
    elems_were_removed: bool
    __started_processing_size: int

    def __init__(self) -> None:
        self.modified = []
        self.elems_were_removed = False

    def _added_elem(self, elem) -> None:
        self.modified.append(elem)
    
    def _removed_elem(self, elem) -> None:
        self.elems_were_removed = True
        self.modified.append(elem)

    def clear_tracking_data(self) -> None:
        self.modified.clear()
        self.elems_were_removed = False

