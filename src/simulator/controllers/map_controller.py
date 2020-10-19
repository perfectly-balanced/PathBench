from simulator.controllers.controller import Controller
from simulator.views.map_view import MapView
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.window_loaded_event import WindowLoadedEvent
from simulator.services.debug import DebugLevel

from structures import Point

from panda3d.core import CollisionTraverser, CollisionHandlerQueue, CollisionNode, BitMask32, CollisionBox, CollisionRay, Point3

from direct.showbase.ShowBase import ShowBase
from simulator.views.panda.voxel_map import VoxelMap

import math
from typing import Optional

class MapTraversablesPicker():
    __base: ShowBase
    __map: VoxelMap


    def __init__(self, base: ShowBase, map: VoxelMap):
        self.__base = base
        self.__map = map

        # collision traverser & queue
        self.__ctrav = CollisionTraverser('ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # map collision boxes
        self.__map_cn = CollisionNode('3D Voxel Map Collision Node')
        self.__map_cn.set_collide_mask(BitMask32.bit(1))
        self.__map_cnp = self.__map.traversables.attach_new_node(self.__map_cn)
        self.__ctrav.add_collider(self.__map_cnp, self.__cqueue)
        
        for x in range(len(self.__map.traversables_data)):
            for y in range(len(self.__map.traversables_data[x])):
                for z in range(len(self.__map.traversables_data[x][y])):
                    if self.__map.traversables_mesh.cube_visible((x, y, z)):
                        self.__map_cn.add_solid(CollisionBox(Point3(x,y,z), Point3(x+1, y+1, z-1)))

        # mouse picker
        picker_node = CollisionNode('mouseRay')
        picker_np = self.__base.cam.attach_new_node(picker_node)
        picker_node.set_from_collide_mask(BitMask32.bit(1))
        
        self.__picker_ray = CollisionRay()
        picker_node.add_solid(self.__picker_ray)
        self.__ctrav.add_collider(picker_np, self.__cqueue)

        # debug -> shows collision ray / impact point
        # self.__ctrav.show_collisions(self.__map.root)

    @property
    def pos(self):
        # check if we have access to the mouse
        if self.__base.mouseWatcherNode.hasMouse():
            # get the mouse position
            mpos = self.__base.mouseWatcherNode.get_mouse()

            # set the position of the ray based on the mouse position
            self.__picker_ray.set_from_lens(self.__base.camNode,mpos.getX(),mpos.getY())
            self.__ctrav.traverse(self.__map.traversables)
            # if we have hit something sort the hits so that the closest is first and highlight the node
            if self.__cqueue.get_num_entries() > 0:
                self.__cqueue.sort_entries()
                po = self.__cqueue.get_entry(0).get_into_node_path()

                x, y, z = self.__cqueue.get_entry(0).getSurfacePoint(self.__map.traversables)
                pos = [max(math.floor(x), 0), max(math.floor(y), 0), max(math.ceil(z), 0)]
                if pos[0] == len(self.__map.traversables_data):
                    pos[0] -= 1
                if pos[1] == len(self.__map.traversables_data[pos[0]]):
                    pos[1] -= 1
                if pos[2] == len(self.__map.traversables_data[pos[0]][pos[1]]):
                    pos[2] -= 1
                pos = Point(*pos)                
                return pos
        return None

class MapController(Controller):
    __picker: Optional[MapTraversablesPicker]
    __view: MapView

    def __init__(self, view: MapView, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__view = view
        self.__picker = None
    
    def __init(self):
        self.__picker = MapTraversablesPicker(self._services.window, self.__view.map)

        def left_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved agent to: " + str(p), DebugLevel.MEDIUM)
                self._model.move(p)
        
        def right_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved goal to: " + str(p), DebugLevel.MEDIUM)
                self._model.move_goal(p)

        base = self._services.window
        base.accept('mouse1', left_click)
        base.accept('mouse3', right_click)
        base.accept("arrow_up", lambda: self._model.move_up())
        base.accept("arrow_down", lambda: self._model.move_down())
        base.accept("arrow_left", lambda: self._model.move_left())
        base.accept("arrow_right", lambda: self._model.move_right())
        base.accept("c", lambda: self._model.compute_trace())
        base.accept("m", lambda: self._model.toggle_convert_map())
        base.accept("x", lambda: self._model.stop_algorithm())
        base.accept("z", lambda: self._model.resume_algorithm())
        base.accept("p", lambda: self._services.ev_manager.post(TakeScreenshotEvent()))

    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, WindowLoadedEvent):
            self.__init()
