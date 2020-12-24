from structures import Point
from utility.compatibility import Final
from simulator.views.map.data.map_data import MapData
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.map_update_event import MapUpdateEvent

import math
from nptyping import NDArray
from typing import Any, List, Optional
import numpy as np

from panda3d.core import CollisionTraverser, CollisionHandlerQueue, CollisionNode, NodePath, BitMask32, CollisionBox, CollisionRay, Point3
from direct.showbase.ShowBase import ShowBase

class MapPicker():
    __name: Final[str]
    __base: Final[ShowBase]
    __data: Final[NDArray[(Any, Any, Any), np.uint8]]

    # collision data
    __ctrav: Final[CollisionTraverser]
    __cqueue: Final[CollisionHandlerQueue]
    __cn: Final[CollisionNode]
    __cnp: Final[NodePath]

    # picker data
    __pn: Final[CollisionNode]
    __pnp: Final[NodePath]
    __pray: Final[CollisionRay]

    # constants
    COLLIDE_MASK: Final[BitMask32] = BitMask32.bit(1)

    def __init__(self, services: Services, base: ShowBase, map_data: MapData, name: Optional[str] = None):
        self.__services = services
        self.__services.ev_manager.register_listener(self)
        self.__base = base
        self.__name = name if name is not None else (map_data.name + "_picker")
        self.__map = map_data
        self.__data = map_data.data

        # collision traverser & queue
        self.__ctrav = CollisionTraverser(self.name + '_ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # collision boxes
        self.__cn = CollisionNode(self.name + '_cn')
        self.__cn.set_collide_mask(MapPicker.COLLIDE_MASK)
        self.__cnp = self.__map.root.attach_new_node(self.__cn)
        self.__ctrav.add_collider(self.__cnp, self.__cqueue)
        self.__points = []

        z_offset = 1 if self.__map.dim == 3 else self.__map.depth
        for idx in np.ndindex(self.__data.shape):
            if bool(self.__data[idx] & MapData.TRAVERSABLE_MASK):
                p = Point(*idx)
                self.__points.append(p)
                idx = self.__cn.add_solid(CollisionBox(idx, Point3(p.x+1, p.y+1, p.z-z_offset)))
                assert idx == (len(self.__points) - 1)

        # mouse picker
        self.__pn = CollisionNode(self.name + '_pray')
        self.__pnp = self.__base.cam.attach_new_node(self.__pn)
        self.__pn.set_from_collide_mask(MapPicker.COLLIDE_MASK)

        self.__pray = CollisionRay()
        self.__pn.add_solid(self.__pray)
        self.__ctrav.add_collider(self.__pnp, self.__cqueue)

        # debug -> shows collision ray / impact point
        # self.__ctrav.show_collisions(self.__map.root)

    @property
    def name(self) -> str:
        return self.__name

    @property
    def pos(self):
        # check if we have access to the mouse
        if not self.__base.mouseWatcherNode.hasMouse():
            return None

        # get the mouse position
        mpos = self.__base.mouseWatcherNode.get_mouse()

        # set the position of the ray based on the mouse position
        self.__pray.set_from_lens(self.__base.camNode, mpos.getX(), mpos.getY())

        # find collisions
        self.__ctrav.traverse(self.__map.root)

        # if we have hit something sort the hits so that the closest is first
        if self.__cqueue.get_num_entries() == 0:
            return None
        self.__cqueue.sort_entries()

        # compute & return logical cube position
        x, y, z = self.__cqueue.get_entry(0).getSurfacePoint(self.__map.root)
        x, y, z = [max(math.floor(x), 0), max(math.floor(y), 0), max(math.ceil(z), 0)]
        if x == len(self.__data):
            x -= 1
        if y == len(self.__data[x]):
            y -= 1
        if z == len(self.__data[x][y]):
            z -= 1

        return Point(x, y, z)

    def notify(self, event: Event) -> None:
        if isinstance(event, MapUpdateEvent):
            z_offset = 1 if self.__map.dim == 3 else self.__map.depth
            for p in event.updated_cells:
                if p.n_dim == 2:
                    p = Point(*p, 0)

                if bool(self.__data[p.values] & MapData.TRAVERSABLE_MASK):
                    self.__points.append(p)
                    idx = self.__cn.add_solid(CollisionBox(p.values, Point3(p.x+1, p.y+1, p.z-z_offset)))
                    assert idx == (len(self.__points) - 1)
                else:
                    try:
                        i = self.__points.index(p)
                    except ValueError:
                        continue
                    self.__cn.remove_solid(i)
                    self.__points.pop(i)

    def destroy(self) -> None:
        self.__cqueue.clearEntries()
        self.__ctrav.clear_colliders()
        self.__cnp.remove_node()
        self.__pnp.remove_node()
