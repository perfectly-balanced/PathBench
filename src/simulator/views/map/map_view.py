from time import sleep

from typing import List, Any, Tuple, Optional, Union, Set
import os

from constants import DATA_PATH

from heapq import heappush, heappop

from algorithms.configuration.entities.entity import Entity
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent

from simulator.views.map.display.entities_map_display import EntitiesMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.numbers_map_display import NumbersMapDisplay
from simulator.views.map.display.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.view import View
from structures import Point, Colour, TRANSPARENT, WHITE

from simulator.views.map.data.voxel_map import VoxelMap
from simulator.views.map.object.cube_mesh import Face

from panda3d.core import NodePath, GeomNode, Geom, LineSegs, TextNode
from direct.showutil import BuildGeometry

import math
import numpy as np

class MapView(View):
    __world: NodePath
    __map: VoxelMap

    __displays: List[Any]
    __persistent_displays: List[Any]
    __tracked_data: List[Any]

    __second_pass_displays: List[Any]
    __cubes_requiring_update: Set[Point]
    __display_updates_cube: bool
    __cube_colour: Colour

    __overlay: NodePath
    __np_collector: Optional[NodePath]
    __draw_nps: List[NodePath]
    __circles: List[Tuple[int, float, Geom]]
    __line_segs: LineSegs    

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self.__displays = []
        self.__tracked_data = []

        self.__second_pass_displays = []
        self.__cubes_requiring_update = set()
        self.__display_updates_cube = False
        self.__cube_colour = None

        self.__np_collector = None
        self.__draw_nps = []
        self.__circles = []
        self.__line_segs = LineSegs()
        self.__line_segs.set_thickness(2)

        # world (dummy node)
        self.__world = self._services.graphics.window.render.attach_new_node("world")

        map_size = self._services.algorithm.map.size
        map_data = np.empty((*map_size, 1) if map_size.n_dim == 2 else map_size, dtype=bool)
        for x, y, z in np.ndindex(map_data.shape):
            p = Point(x, y) if map_size.n_dim == 2 else Point(x, y, z)
            map_data[x, y, z] = not self._services.algorithm.map.is_agent_valid_pos(p)

        # MAP #
        self.__map = VoxelMap(self._services, map_data, self.world, artificial_lighting=True)
        self.__center(self.__map.root)

        self.__overlay = self.map.root.attach_new_node("overlay")
        self.__persistent_displays = [EntitiesMapDisplay(self._services)]

        self.update_view(True)

    def destroy(self) -> None:
        self.__map.destroy()
        self.__world.remove_node()
        self.__displays = []
        self.__persistent_displays = []
        self.__draw_nps = []
        for _, _, geo in self.__circles:
            geo.releaseAll()
        self.__circles = []
        self._services.ev_manager.unregister_listener(self)
        self._services.ev_manager.unregister_tick_listener(self)
        if self._root_view is not None:
            self._root_view.remove_child(self)

    def __refresh(self) -> None:
        self.__overlay.flatten_strong()
        self.__overlay.remove_node()
        self.__overlay = self.map.root.attach_new_node("overlay")

    def notify(self, event: Event) -> None:
        super().notify(event)
        if isinstance(event, KeyFrameEvent):
            if event.refresh:
                self.__refresh()
            self.update_view(event.refresh)

        elif isinstance(event, ColourUpdateEvent):
            if event.view.is_effective():
                self.update_view(False)
        elif isinstance(event, TakeScreenshotEvent):
            self.take_screenshot()

    def to_point3(self, v: Union[Point, Entity]):
        if isinstance(v, Entity):
            v = v.position
        return Point(*v, 0) if len(v) == 2 else v

    def __center(self, np: NodePath):
        (x1, y1, z1), (x2, y2, z2) = np.get_tight_bounds()
        np.set_pos(self.world.getX() - (x2 - x1) / 2, self.world.getY() - (y2 - y1) / 2,
                   self.world.getZ() - (z2 - z1) / 2)

    @property
    def world(self) -> str:
        return 'world'

    @world.getter
    def world(self) -> NodePath:
        return self.__world

    @property
    def map(self) -> str:
        return 'map'

    @map.getter
    def map(self) -> VoxelMap:
        return self.__map

    @property
    def overlay(self) -> NodePath:
        return self.__overlay

    def __get_displays(self) -> None:
        # drop map displays if not compatible with display format
        displays: List[MapDisplay] = list(
            filter(lambda d: self._services.settings.simulator_grid_display or not isinstance(d, NumbersMapDisplay),
                   self._services.algorithm.instance.get_display_info()))
        displays += self.__persistent_displays

        for display in displays:
            display._model = self._model
            self.add_child(display)
            heappush(self.__displays, display)
            self.__tracked_data += display.get_tracked_data()

    def __render_displays(self, refresh: bool) -> None:
        for np in self.__draw_nps:
            np.remove_node()
        self.__draw_nps.clear()
        
        while len(self.__displays) > 0:
            display: MapDisplay
            display = heappop(self.__displays)

            display.render(refresh)
            if self.__display_updates_cube:
                self.__display_updates_cube = False
                self.__second_pass_displays.append(display)
            
            self.__render_lines()

    def __update_cubes(self, refresh: bool) -> None:
        clr = self._services.state.effective_view.colours["traversables"]()
        mesh = self.map.traversables_mesh

        def update_cube_colour(p):
            self.__cube_colour = clr
            for d in self.__second_pass_displays:
                d.update_cube(p)
            mesh.set_cube_colour(p, self.__cube_colour)
        
        if refresh:
            for x, y, z in np.ndindex(mesh.structure.shape):
                if mesh.structure[x, y, z]:
                    update_cube_colour(Point(x, y, z))
        else:
            for p in self.__cubes_requiring_update:
                update_cube_colour(p)
        
        self.__second_pass_displays.clear()
        self.__cubes_requiring_update.clear()

        for td in self.__tracked_data:
            td.clear_tracking_data()
        
        self.__tracked_data.clear()
        self.__displays.clear()

    def update_view(self, refresh: bool) -> None:
        self._services.lock.acquire()
        self.__get_displays()
        self.__render_displays(refresh)
        self.__update_cubes(refresh)
        self._services.lock.release()

    def display_updates_cube(self) -> None:
        self.__display_updates_cube = True

    def cube_requires_update(self, v: Union[Point, Entity]) -> Point:
        p = self.to_point3(v)
        self.__cubes_requiring_update.add(p)
        return p

    def colour_cube(self, src: Colour) -> None:
        dst = self.__cube_colour

        wda = dst.a * (1 - src.a)  # weighted dst alpha
        a = src.a + wda
        d = (a if a != 0 else 1)
        r = (src.r * src.a + dst.r * wda) / d
        g = (src.g * src.a + dst.g * wda) / d
        b = (src.b * src.a + dst.b * wda) / d

        self.__cube_colour = Colour(r, g, b, a)

    def take_screenshot(self) -> None:
        self._services.resources.screenshots_dir.append(
            lambda fn: self._services.graphics.window.win.save_screenshot(fn))

    def cube_center(self, p: Point) -> Point:
        x, y, z = self.to_point3(p)
        x += 0.5
        y += 0.5
        if self._services.algorithm.map.size.n_dim == 3:
            z -= 0.5
        else:
            z = 0
        return Point(x, y, z)

    def start_collecting_nodes(self, np: Optional[NodePath] = None) -> None:
        self.__render_lines()
        self.__np_collector = self.overlay.attach_new_node('collection') if np is None else np

    def end_collecting_nodes(self, flatten: bool = True) -> NodePath:
        self.__render_lines()
        np = self.__np_collector
        self.__np_collector = None
        if flatten:
            np.flatten_strong()
        return np

    def __render_lines(self) -> None:
        if not self.__line_segs.is_empty():
            n = self.__line_segs.create()
            if self.__np_collector is None:
                np = self.overlay.attach_new_node(n)
                self.__draw_nps.append(np)
            else:
                self.__np_collector.attach_new_node(n)

    def draw_line(self, colour: Colour, p1: Point, p2: Point) -> None:
        ls = self.__line_segs
        ls.set_color(*colour)

        ls.move_to(*self.cube_center(p1))
        ls.draw_to(*self.cube_center(p2))

    def draw_sphere(self, p: Point, colour: Colour = WHITE, scale: float = 0.2) -> None:
        np = loader.load_model("models/misc/sphere.egg")
        np.set_color(*colour)

        if self.__np_collector is None:
            np.reparent_to(self.overlay)
            self.__draw_nps.append(np)
        else:
            np.reparent_to(self.__np_collector)

        np.set_pos(*self.cube_center(p))
        np.set_scale(scale)

    def make_arc(self, p: Point, angle_degs=360, nsteps=16, radius: float = 0.06, colour: Colour = WHITE) -> None:
        ls = self.__line_segs
        ls.set_color(*colour)

        angle_rads = angle_degs * (math.pi / 180.0)
        x, y, z = self.cube_center(p)

        ls.move_to(x + radius, y, z)
        for i in range(nsteps + 1):
            a = angle_rads * i / nsteps
            ty = math.sin(a) * radius + y
            tx = math.cos(a) * radius + x
            ls.draw_to(tx, ty, z)

    def draw_circle(self, p: Point, *args, **kwargs) -> None:
        self.make_arc(p, 360, *args, **kwargs)

    def draw_circle_filled(self, p: Point, nsteps=16, radius: float = 0.06, colour: Colour = WHITE) -> None:
        gn = GeomNode("circle")

        exists: bool = False
        for cn, cr, cg in self.__circles:
            if cn == nsteps and cr == radius:
                exists = True
                gn.add_geom(cg)
                break
        if not exists:
            self.__circles.append((nsteps, radius, BuildGeometry.addCircle(gn, nsteps, radius, colour)))

        if self.__np_collector is None:
            np = self.overlay.attach_new_node(gn)
            self.__draw_nps.append(np)
        else:
            np = self.__np_collector.attach_new_node(gn)
        
        np.set_pos(*self.cube_center(p))
        np.set_color(*colour)

    def render_text(self, p: Point, text: str, colour: Colour = WHITE, scale: float = 0.4) -> None:
        center = self.cube_center(p)

        offset, hpr = Point(-0.25, 0, 0), (0, -90, 0)

        n = TextNode('text')
        n.set_text(text)
        np = self.overlay.attach_new_node(n)
        np.set_scale(scale)
        np.set_color(*colour)
        np.set_pos(*(center + offset))
        np.set_hpr(*hpr)

        self.__draw_nps.append(np)
