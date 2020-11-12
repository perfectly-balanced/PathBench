from constants import DATA_PATH
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.take_screenshot_tex_event import TakeScreenshotTexEvent
from simulator.services.graphics.renderer import Renderer
from simulator.views.map.display.entities_map_display import EntitiesMapDisplay
from simulator.views.map.display.solid_colour_map_display import SolidColourMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.numbers_map_display import NumbersMapDisplay
from simulator.views.map.display.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.map.data.map_data import MapData
from simulator.views.map.data.voxel_map import VoxelMap
from simulator.views.map.data.flat_map import FlatMap
from simulator.views.util import blend_colours
from simulator.views.view import View

from structures import Point, Colour, TRANSPARENT, WHITE
from structures.tracked_set import TrackedSet

from panda3d.core import Camera, Texture, NodePath, GeomNode, Geom, LineSegs, TextNode, PandaNode

import numpy as np
from nptyping import NDArray
from typing import List, Any, Tuple, Optional, Union, Set
from heapq import heappush, heappop
import os


class MapView(View):
    __world: NodePath
    __map: Union[VoxelMap, FlatMap]

    __displays: List[Any]
    __persistent_displays: List[Any]
    __tracked_data: List[Any]

    __cube_modified: NDArray[(Any, Any, Any), bool]
    __cube_update_displays: List[Any]
    __cubes_requiring_update: Set[Point]
    __display_updates_cube: bool
    __cube_colour: Colour

    __scratch: NodePath
    __overlay: NodePath

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self.__displays = []
        self.__tracked_data = []

        self.__cube_update_displays = []
        self.__cubes_requiring_update = set()
        self.__display_updates_cube = False
        self.__cube_colour = None

        # world (dummy node)
        self.__world = self._services.graphics.window.render.attach_new_node("world")

        # MAP #
        map_size = self._services.algorithm.map.size
        map_data = np.empty((*map_size, 1) if map_size.n_dim == 2 else map_size, dtype=bool)
        self.__cube_modified = np.empty(map_data.shape, dtype=bool)
        for x, y, z in np.ndindex(map_data.shape):
            p = Point(x, y) if map_size.n_dim == 2 else Point(x, y, z)
            is_wall = self._services.algorithm.map.at(p) == Map.WALL_ID
            self.__cube_modified[x, y, z] = not is_wall
            map_data[x, y, z] = is_wall

        if map_size.n_dim == 2:
            self.__map = FlatMap(self._services, map_data, self.world)
        else:
            self.__map = VoxelMap(self._services, map_data, self.world, artificial_lighting=True)
        self.__map.center()

        self.__overlay = self.map.root.attach_new_node("overlay")
        self.__scratch = self.map.root.attach_new_node("scratch")
        self.renderer.push_root(self.__scratch)

        self.__persistent_displays = [EntitiesMapDisplay(self._services)]

        extended_walls = TrackedSet()
        for x, y, z in np.ndindex(map_data.shape):
            p = Point(x, y) if map_size.n_dim == 2 else Point(x, y, z)
            if self._services.algorithm.map.at(p) == Map.EXTENDED_WALL:
                extended_walls.add(Point(x, y, z)) # using 3D points is more efficient
        if extended_walls:
            dc = self._services.state.add_colour("extended wall", Colour(0.5).with_a(0.5))
            self.__persistent_displays.append(SolidColourMapDisplay(self._services, extended_walls, dc, z_index=0))

        self.__deduced_traversables_colour = self._services.state.effective_view.colours[MapData.TRAVERSABLES]()
        self.__deduced_traversables_wf_colour = self._services.state.effective_view.colours[MapData.TRAVERSABLES_WF]()

        self.__sphere_scale = 0.2
        """
        def rescale_sphere(dim):
            while (self.__sphere_scale < 0.75) and ((self.__sphere_scale * dim) < 8):
                self.__sphere_scale *= 1.25
        rescale_sphere(self.map.logical_w)
        rescale_sphere(self.map.logical_h)
        rescale_sphere(self.map.logical_d)
        """

        self.__circle_filled_radius = 0.06

        def resize_circle_filled(dim):
            if dim < 40:
                pass
            elif dim < 75:
                dim /= 1.5
            elif dim < 100:
                dim /= 2
            while (self.__circle_filled_radius < 1) and (dim / self.__circle_filled_radius > 250):
                self.__circle_filled_radius *= 1.25

        resize_circle_filled(self.map.logical_w)
        resize_circle_filled(self.map.logical_h)
        resize_circle_filled(self.map.logical_d)

        self.__line_thickness = 2.5

        def change_line_thickness(dim):
            while (self.__line_thickness < 4) and (dim / self.__line_thickness > 160):
                self.__line_thickness *= 1.25

        change_line_thickness(self.map.logical_w)
        change_line_thickness(self.map.logical_h)
        change_line_thickness(self.map.logical_d)
        self.renderer.line_segs.set_thickness(self.__line_thickness)

        self.update_view(True)

    def destroy(self) -> None:
        self.__map.destroy()
        self.__world.remove_node()
        self.__displays = []
        self.__persistent_displays = []
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
        elif isinstance(event, TakeScreenshotTexEvent):
            self.HDScreenShot()

    def to_logical_point(self, p: Point) -> Point:
        x, y, z = p
        return Point(x, y) if self.map.dim == 2 else Point(x, y, z)

    def to_point3(self, v: Union[Point, Entity]) -> Point:
        if isinstance(v, Entity):
            v = v.position
        return Point(*v, 0) if len(v) == 2 else v

    @property
    def world(self) -> NodePath:
        return self.__world

    @property
    def map(self) -> Union[VoxelMap, FlatMap]:
        return self.__map

    @property
    def overlay(self) -> NodePath:
        return self.__overlay

    @property
    def renderer(self) -> Renderer:
        return self._services.graphics.renderer

    def __clear_scratch(self) -> None:
        assert self.renderer.root == self.__scratch
        for c in self.__scratch.get_children():
            c.remove_node()

    def __get_displays(self) -> None:
        def add_diplay(d):
            d._model = self._model
            self.add_child(d)
            heappush(self.__displays, d)
            self.__tracked_data += d.get_tracked_data()

        for d in self.__persistent_displays:
            add_diplay(d)
        for d in self._services.algorithm.instance.get_display_info():
            add_diplay(d)

    def __render_displays(self, refresh: bool) -> None:
        while len(self.__displays) > 0:
            display: MapDisplay
            display = heappop(self.__displays)

            display.render(refresh)
            if self.__display_updates_cube:
                self.__display_updates_cube = False
                self.__cube_update_displays.append(display)

    def __update_cubes(self, refresh: bool) -> None:
        def eager_refresh():
            nonlocal refresh
            refresh = True
            for p in np.ndindex(self.map.traversables_data.shape):
                self.__cube_modified[p] = self.map.traversables_data[p]
        
        clr = self._services.state.effective_view.colours[MapData.TRAVERSABLES]()
        if clr != self.__deduced_traversables_colour:
            eager_refresh()
        self.__deduced_traversables_colour = clr

        if self.map.dim == 3:
            def set_colour(p, c):
                return self.map.traversables_mesh.set_cube_colour(p, c)
        else:  # 2D
            wfc = self.map.traversables_wf_dc()
            wfc = self._services.state.effective_view.colours[MapData.TRAVERSABLES_WF]()
            if wfc != self.__deduced_traversables_wf_colour:
                eager_refresh()
            self.__deduced_traversables_wf_colour = wfc

            def set_colour(p, c):
                return self.map.render_square(p, c, wfc)

        def update_cube_colour(p):
            self.__cube_colour = clr
            for d in self.__cube_update_displays:
                d.update_cube(p)
            set_colour(p, self.__cube_colour)
            self.__cube_modified[tuple(p)] = self.__cube_colour != clr

        if refresh:
            for p in np.ndindex(self.__cube_modified.shape):
                if self.__cube_modified[p]:
                    point = Point(*p)
                    update_cube_colour(point)

                    # we don't want to track extended walls once rendered.
                    # Note, they will still be refreshed when traversable
                    # (wireframe) colour changes.
                    if self._services.algorithm.map.at(self.to_logical_point(point)) == Map.EXTENDED_WALL:
                        self.__cube_modified[p] = False
                        self.__cubes_requiring_update.discard(point)
        
        # update these cubes regardless of refresh
        # since it cubes that require update aren't
        # necessarily modified.
        for p in self.__cubes_requiring_update:
            update_cube_colour(p)

        for td in self.__tracked_data:
            td.clear_tracking_data()

        self.__tracked_data.clear()
        self.__cubes_requiring_update.clear()
        self.__cube_update_displays.clear()
        self.__displays.clear()

    def update_view(self, refresh: bool) -> None:
        self.__clear_scratch()
        self.__get_displays()
        self.__render_displays(refresh)
        self.__update_cubes(refresh)
        self.renderer.render()

    def display_updates_cube(self) -> None:
        self.__display_updates_cube = True

    def cube_requires_update(self, v: Union[Point, Entity]) -> Point:
        p = self.to_point3(v)
        self.__cubes_requiring_update.add(p)
        return p

    def colour_cube(self, src: Colour) -> None:
        self.__cube_colour = blend_colours(src, self.__cube_colour)

    def take_screenshot(self) -> None:
        self._services.resources.screenshots_dir.append(
            lambda fn: self._services.graphics.window.win.save_screenshot(fn))

    def HDScreenShot(self):
        a, b, c = self.__map.traversables_data.shape
        # find the optimal zoom fit
        max_width = max(a, b)
        max_height = max(a, c)
        tex = Texture()
        width = 4096
        height = 4096
        mybuffer = self._services.graphics.window.win.makeTextureBuffer('HDScreenshotBuff', width, height, tex, True)

        cam = Camera('HDCam')
        cam.setLens(self._services.graphics.window.camLens.makeCopy())
        cam.getLens().setAspectRatio(width / height)
        npCam = NodePath(cam)
        npCam.reparentTo(self.__world)
        x, y, z = self.__world.getPos()
        print(c)
        # 3d case

        if c > 1:
            print(x,y,z)

            npCam.setPos(x-a * 2.1, y - max_height * 3.6, z)
            npCam.setH(-30)
        else:
            npCam.setPos(x, y, z + max_width * 2 + c)
            npCam.setP(-90)

        mycamera = self._services.graphics.window.makeCamera(mybuffer, useCamera=npCam)
        myscene = self._services.graphics.window.render
        mycamera.node().setScene(myscene)
        self._services.graphics.window.graphicsEngine.renderFrame()
        tex = mybuffer.getTexture()
        mybuffer.setActive(False)

        self._services.resources.screenshots_dir.append(
        lambda fn: tex.write(fn))
        self._services.graphics.window.graphicsEngine.removeWindow(mybuffer)
        print("HDScreenShot taken")

    def cube_center(self, p: Point) -> Point:
        x, y, z = self.to_point3(p)
        return Point(x + 0.5, y + 0.5, z - 0.5)

    def push_root(self, np: Optional[NodePath] = None) -> NodePath:
        if np is None:
            np = self.overlay.attach_new_node('collection')
        self.renderer.push_root(np)
        return np

    def pop_root(self) -> NodePath:
        return self.renderer.pop_root()

    def draw_line(self, colour: Colour, p1: Point, p2: Point) -> None:
        self.renderer.draw_line(colour, self.cube_center(p1), self.cube_center(p2))

    def draw_sphere(self, p: Point, *args, **kwargs) -> None:
        self.renderer.draw_sphere(self.cube_center(p), *args, **kwargs, scale=self.__sphere_scale)

    def make_arc(self, p: Point, *args, **kwargs) -> None:
        self.renderer.make_arc(self.cube_center(p), *args, **kwargs)

    def draw_circle(self, p: Point, *args, **kwargs) -> None:
        self.make_arc(p, 360, *args, **kwargs)

    def draw_circle_filled(self, p: Point, *args, **kwargs) -> None:
        self.renderer.draw_circle_filled(self.cube_center(p), *args, **kwargs, radius=self.__circle_filled_radius)

    def render_text(self, p: Point, text: str, colour: Colour = WHITE, scale: float = 0.4) -> None:
        """ TODO: reintergrate into reworked code """
        center = self.cube_center(p)

        offset, hpr = Point(-0.25, 0, 0), (0, -90, 0)

        n = TextNode('text')
        n.set_text(text)
        np = self.overlay.attach_new_node(n)
        np.set_scale(scale)
        np.set_color(*colour)
        np.set_pos(*(center + offset))
        np.set_hpr(*hpr)
