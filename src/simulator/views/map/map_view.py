from time import sleep

from heapq import heappush, heappop
from typing import List, Any, Tuple, Optional, Union
import os

from constants import DATA_PATH

from algorithms.configuration.entities.entity import Entity
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent

from simulator.views.map_displays.entities_map_display import EntitiesMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from simulator.views.map_displays.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.view import View
from structures import Point, Colour, TRANSPARENT, WHITE

from simulator.views.gui.view_editor import ViewEditor
from simulator.views.map.data.voxel_map import VoxelMap
from simulator.views.map.object.cube_mesh import Face

from panda3d.core import NodePath, GeomNode, Geom, LineSegs, TextNode
from direct.showutil import BuildGeometry

import math


class MapView(View):
    __displays: List[Any]

    __world: NodePath
    __map: VoxelMap
    __vs: ViewEditor

    __tc_previous: List[List[List[Colour]]]
    __tc_scratchpad: List[List[List[Colour]]]
    __draw_nps: List[NodePath]
    __circles: List[Tuple[int, float, Geom]]
    __line_segs: LineSegs
    __angle_degs: float
    __angle_rads: float
    __nsteps: int
    __thickness: float
    __rad = float
    __colour: Colour

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self.__displays = []
        self.__tc_previous = {}
        self.__tc_scratchpad = {}
        self.__draw_nps = []
        self.__circles = []
        self.__line_segs = LineSegs()
        self.__line_segs.set_thickness(2)

        # world (dummy node)
        self.__world = self._services.graphics.window.render.attach_new_node("world")

        map_data = {}
        for x in range(0, self._services.algorithm.map.size.width):
            map_data[x] = {}
            for y in range(0, self._services.algorithm.map.size.height):
                map_data[x][y] = {}
                if self._services.algorithm.map.size.n_dim == 2:
                    map_data[x][y][0] = not self._services.algorithm.map.is_agent_valid_pos(Point(x, y))
                else:
                    # assume 3D
                    for z in range(0, self._services.algorithm.map.size.depth):
                        map_data[x][y][z] = not self._services.algorithm.map.is_agent_valid_pos(Point(x, y, z))

        # MAP #
        self.__map = VoxelMap(self._services, map_data, self.world, artificial_lighting=True)
        self.__center(self.__map.root)

        # GUI #
        self.__vs = ViewEditor(self._services, self.map)

        self.update_view()

    def notify(self, event: Event) -> None:
        super().notify(event)
        if isinstance(event, KeyFrameEvent):
            if event.refresh:
                self.__tc_reset()
            self.update_view()
        elif isinstance(event, TakeScreenshotEvent):
            self.take_screenshot()

    def __to_point3(self, v: Union[Point, Entity]):
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

    def __tc_reset(self) -> None:
        self.__tc_scratchpad = {}
        self.__tc_previous = {}
        self.map.traversables_mesh.reset_all_cubes()

    def __get_displays(self) -> None:
        self.__displays = []
        displays: List[MapDisplay] = [EntitiesMapDisplay(self.__map, self._services)]
        # drop map displays if not compatible with display format
        displays += list(
            filter(lambda d: self._services.settings.simulator_grid_display or not isinstance(d, NumbersMapDisplay),
                   self._services.algorithm.instance.get_display_info()))
        for display in displays:
            display._model = self._model
            self.add_child(display)
            heappush(self.__displays, (display.z_index, display))
    
    def __render_displays(self) -> None:
        for np in self.__draw_nps:
            np.remove_node()
        self.__draw_nps.clear()
        while len(self.__displays) > 0:
            display: MapDisplay
            _, display = heappop(self.__displays)
            display.render()
            if not self.__line_segs.is_empty():
                n = self.__line_segs.create()
                np = self.map.root.attach_new_node(n)
                self.__draw_nps.append(np)

        # update actual viewable map data
        # lazily update mesh data, maybe this is actually slower
        """
        for x in self.__tc_scratchpad:
            if x in self.__tc_previous:
                for y in self.__tc_scratchpad[x]:
                    if y in self.__tc_previous[x]:
                        for z in self.__tc_scratchpad[x][y]:
                            if z in self.__tc_previous[x][y]:
                                def approx_eq(c1, c2):
                                    r1, g1, b1, a1 = c1
                                    r2, g2, b2, a2 = c2
                                    TOLERANCE = 1e-3
                                    return math.isclose(r1, r2, rel_tol=TOLERANCE) and \
                                           math.isclose(g1, g2, rel_tol=TOLERANCE) and \
                                           math.isclose(b1, b2, rel_tol=TOLERANCE) and \
                                           math.isclose(a1, a2, rel_tol=TOLERANCE)

                                if not approx_eq(self.__tc_previous[x][y][z], self.__tc_scratchpad[x][y][z]):
                                    self.map.traversables_mesh.set_cube_colour(Point(x, y, z), self.__tc_scratchpad[x][y][z])
                            else:
                                self.map.traversables_mesh.set_cube_colour(Point(x, y, z), self.__tc_scratchpad[x][y][z])
                    else:
                        for z in self.__tc_scratchpad[x][y]:
                            self.map.traversables_mesh.set_cube_colour(Point(x, y, z), self.__tc_scratchpad[x][y][z])
            else:
                for y in self.__tc_scratchpad[x]:
                    for z in self.__tc_scratchpad[x][y]:
                        self.map.traversables_mesh.set_cube_colour(Point(x, y, z), self.__tc_scratchpad[x][y][z])
        """
        for x in self.__tc_scratchpad:
            for y in self.__tc_scratchpad[x]:
                for z in self.__tc_scratchpad[x][y]:
                    self.map.traversables_mesh.set_cube_colour(Point(x, y, z), self.__tc_scratchpad[x][y][z])

        # for our purposes a simple reset on first frame may suffice
        # i.e. may not need to do the following on each frame
        for x in self.__tc_previous:
            should_reset = x not in self.__tc_scratchpad
            for y in self.__tc_previous[x]:
                if not should_reset:
                    should_reset = y not in self.__tc_scratchpad[x]
                for z in self.__tc_previous[x][y]:
                    if not should_reset:
                        should_reset = z not in self.__tc_scratchpad[x][y]
                    if should_reset:
                        self.map.traversables_mesh.reset_cube(Point(x, y, z))

        self.__tc_previous = self.__tc_scratchpad
        self.__tc_scratchpad = {}

    def render_pos(self, pe: Union[Point, Entity], src: Colour) -> None:
        x, y, z = self.__to_point3(pe)
        if x not in self.__tc_scratchpad:
            self.__tc_scratchpad[x] = {}
        if y not in self.__tc_scratchpad[x]:
            self.__tc_scratchpad[x][y] = {}
        if z not in self.__tc_scratchpad[x][y]:
            self.__tc_scratchpad[x][y][z] = self.map.colours["traversables"].colour # use raw colour
        dst = self.__tc_scratchpad[x][y][z]

        wda = dst.a * (1 - src.a)  # weighted dst alpha
        a = src.a + wda
        r = (src.r * src.a + dst.r * wda) / a
        g = (src.g * src.a + dst.g * wda) / a
        b = (src.b * src.a + dst.b * wda) / a

        self.__tc_scratchpad[x][y][z] = Colour(r, g, b, a)

    def update_view(self) -> None:
        self.__get_displays()
        self.__render_displays()

    def take_screenshot(self) -> None:
        self._services.resources.screenshots_dir.append(
            lambda fn: self._services.graphics.window.win.save_screenshot(fn))

    def cube_center(self, p: Point) -> Point:
        x, y, z = self.__to_point3(p)
        x += 0.5
        y += 0.5
        if self._services.algorithm.map.size.n_dim == 3:
            z -= 0.5
        else:
            z = 0
        return Point(x, y, z)

    @property
    def line_segs(self) -> LineSegs:
        self.__line_segs_used = True
        return self.__line_segs

    def draw_line(self, colour: Colour, p1: Point, p2: Point) -> None:
        ls = self.line_segs
        ls.set_color(*colour)

        ls.move_to(*self.cube_center(p1))
        ls.draw_to(*self.cube_center(p2))

    def draw_sphere(self, p: Point, colour: Colour = WHITE) -> None:
        x, y, z = self.cube_center(p)
        sphere = loader.load_model("models/misc/sphere.egg")
        sphere.set_color(*colour)
        sphere.reparent_to(self.map.root)
        sphere.set_scale(0.2)
        sphere.set_pos(x, y, z)
    
    def make_arc(self, p: Point, angle_degs = 360, nsteps = 16, radius: float = 0.06, colour: Colour = WHITE) -> None:
        ls = self.line_segs
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

    def draw_circle_filled(self, p: Point, nsteps = 16, radius: float = 0.06, colour: Colour = WHITE) -> None:
        gn = GeomNode("circle")
        
        exists: bool = False
        for cn, cr, cg in self.__circles:
            if cn == nsteps and cr == radius:
                exists = True
                gn.add_geom(cg)
                break
        if not exists:
            self.__circles.append((nsteps, radius, BuildGeometry.addCircle(gn, nsteps, radius, colour)))

        np = self.map.root.attach_new_node(gn)
        np.set_pos(*self.cube_center(p))
        np.set_color(*colour)

        self.__draw_nps.append(np)
    
    def render_text(self, p: Point, text: str, colour: Colour = WHITE, scale: float = 0.4) -> None:
        center = self.cube_center(p)

        offset, hpr = Point(-0.25, 0, 0), (0,-90,0)

        n = TextNode('text')
        n.set_text(text)
        np = self.map.root.attach_new_node(n)
        np.set_scale(scale)
        np.set_color(*colour)
        np.set_pos(*(center + offset))
        np.set_hpr(*hpr)

        self.__draw_nps.append(np)