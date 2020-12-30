from simulator.services.services import Services

from structures.heap import Heap
from structures.tracked_heap import TrackedHeap
from structures.tracked_list import TrackedList
from structures.tracked_set import TrackedSet
from structures.tracked_grid import TrackedGrid

from typing import Any, List, Set, Union

import numpy as np

def prefer_tracked(services: Services) -> bool:
    # graphics with animation (no animation would only be rendered once at the end)
    return services.graphics and services.settings.simulator_key_frame_speed != 0

def gen_heap(services: Services, *args, **kwargs) -> Heap:
    if prefer_tracked(services):
        return TrackedHeap(*args, **kwargs)
    else:
        return Heap(*args, **kwargs)

def gen_list(services: Services, *args, **kwargs) -> List[Any]:
    if prefer_tracked(services):
        return TrackedList(*args, **kwargs)
    else:
        return list(*args, **kwargs)

def gen_set(services: Services, *args, **kwargs) -> Set[Any]:
    if prefer_tracked(services):
        return TrackedSet(*args, **kwargs)
    else:
        return set(*args, **kwargs)

def gen_grid(services: Services, *args, **kwargs) -> Union[np.ndarray, TrackedGrid]:
    if prefer_tracked(services):
        return TrackedGrid(*args, **kwargs)
    else:
        return np.array(*args, **kwargs)
