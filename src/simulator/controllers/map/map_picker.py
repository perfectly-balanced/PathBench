from structures import Point
from utility.compatibility import Final
from simulator.views.map.data.map_data import MapData

import math
from nptyping import NDArray
from typing import Any, List, Optional

from panda3d.core import CollisionTraverser, CollisionHandlerQueue, CollisionNode, NodePath, BitMask32, CollisionBox, CollisionRay, Point3
from direct.showbase.ShowBase import ShowBase

class MapPicker():
    __name: Final[str]
    __base: Final[ShowBase]
    __data: Final[NDArray[(Any, Any, Any), bool]]

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

    def __init__(self, base: ShowBase, map_data: MapData, name: Optional[str] = None):
        self.__base = base
        self.__name = name if name is not None else (map_data.name + "_picker")
        self.__map = map_data
        self.__data = map_data.traversables_data

        # collision traverser & queue
        self.__ctrav = CollisionTraverser(self.name + '_ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # collision boxes
        self.__cn = CollisionNode(self.name + '_cn')
        self.__cn.set_collide_mask(MapPicker.COLLIDE_MASK)
        self.__cnp = self.__map.root.attach_new_node(self.__cn)
        self.__ctrav.add_collider(self.__cnp, self.__cqueue)

        z_offset = 1 if self.__map.dim == 3 else self.__map.depth
        for x in range(len(self.__data)):
            for y in range(len(self.__data[x])):
                for z in range(len(self.__data[x][y])):
                    if self.__data[x][y][z]:
                        self.__cn.add_solid(CollisionBox(Point3(x, y, z), Point3(x+1, y+1, z-z_offset)))

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
        if self.__map.dim == 2 and math.isclose(z, -self.__map.depth, rel_tol=1e-3):
            return None  # no picking from underneath
        pos = [max(math.floor(x), 0), max(math.floor(y), 0), max(math.ceil(z), 0)]
        if pos[0] == len(self.__data):
            pos[0] -= 1
        if pos[1] == len(self.__data[pos[0]]):
            pos[1] -= 1
        if pos[2] == len(self.__data[pos[0]][pos[1]]):
            pos[2] -= 1
        pos = Point(*pos)
        return pos

    def destroy(self) -> None:
        self.__cqueue.clearEntries()
        self.__ctrav.clear_colliders()
        self.__cnp.remove_node()
        self.__pnp.remove_node()
