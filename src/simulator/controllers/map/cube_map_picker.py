from typing import Final, List

from panda3d.core import CollisionTraverser, CollisionHandlerQueue, CollisionNode, NodePath, BitMask32, CollisionBox, CollisionRay, Point3
from direct.showbase.ShowBase import ShowBase

from structures import Point

class CubeMapPicker():
    __name: str
    __base: ShowBase
    __data: List[List[List[bool]]]
    __np: NodePath
    
    # collision data
    __ctrav: CollisionTraverser
    __cqueue: CollisionHandlerQueue
    __cn: CollisionNode
    __cnp: NodePath

    # picker data
    __pn: CollisionNode
    __pnp: NodePath
    __pray: CollisionRay

    # constants
    COLLIDE_MASK: Final = BitMask32.bit(1)

    def __init__(self, base: ShowBase, np: NodePath, data: List[List[List[bool]]], name: str = "cube_map_picker"):    
        self.__base = base
        self.__name = name
        self.__data = data
        self.__np = np

        # collision traverser & queue
        self.__ctrav = CollisionTraverser(self.name + '_ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # collision boxes
        self.__cn = CollisionNode(self.name + '_cn')
        self.__cn.set_collide_mask(CubeMapPicker.COLLIDE_MASK)
        self.__cnp = self.__np.attach_new_node(self.__cn)
        self.__ctrav.add_collider(self.__cnp, self.__cqueue)
        
        for x in range(len(self.__data)):
            for y in range(len(self.__data[x])):
                for z in range(len(self.__data[x][y])):
                    if self.__data[x][y][z]:
                        self.__cn.add_solid(CollisionBox(Point3(x,y,z), Point3(x+1, y+1, z-1)))

        # mouse picker
        self.__pn = CollisionNode(self.name + '_pray')
        self.__pnp = self.__base.cam.attach_new_node(self.__pn)
        self.__pn.set_from_collide_mask(CubeMapPicker.COLLIDE_MASK)
        
        self.__pray = CollisionRay()
        self.__pn.add_solid(self.__pray)
        self.__ctrav.add_collider(self.__pnp, self.__cqueue)

        # debug -> shows collision ray / impact point
        # self.__ctrav.show_collisions(self.__np)

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
        self.__pray.set_from_lens(self.__base.camNode,mpos.getX(),mpos.getY())
        
        # find collisions
        self.__ctrav.traverse(self.__np)
        
        # if we have hit something sort the hits so that the closest is first
        if self.__cqueue.get_num_entries() == 0:
            return None
        self.__cqueue.sort_entries()

        # compute & return logical cube position
        x, y, z = self.__cqueue.get_entry(0).getSurfacePoint(self.__np)
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
        self.__np.remove_node()
        self.__cnp.remove_node()
        self.__pnp.remove_node()