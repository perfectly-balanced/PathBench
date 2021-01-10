from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.occupancy_grid_map import OccupancyGridMap
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.take_map_screenshot_event import TakeMapScreenshotEvent
from simulator.services.event_manager.events.map_update_event import MapUpdateEvent
from simulator.services.event_manager.events.state_initialised_event import StateInitialisedEvent
from simulator.services.graphics.renderer import Renderer
from simulator.views.map.display.gradient_list_map_display import GradientListMapDisplay
from simulator.views.map.display.gradient_grid_map_display import GradientGridMapDisplay
from simulator.views.map.display.entities_map_display import EntitiesMapDisplay
from simulator.views.map.display.solid_grid_map_display import SolidGridMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.numbers_map_display import NumbersMapDisplay
from simulator.views.map.display.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.map.data.map_data import MapData
from simulator.views.map.data.voxel_map import VoxelMap
from simulator.views.map.data.flat_map import FlatMap
from simulator.views.util import blend_colours
from simulator.views.view import View

from structures import Point, Colour, TRANSPARENT, WHITE, BLACK
from structures.tracked_set import TrackedSet
from structures.tracked_list import TrackedList
from structures.tracked_grid import TrackedGrid

from panda3d.core import Camera, Texture, NodePath, GeomNode, Geom, LineSegs, TextNode, PandaNode

import numpy as np
from nptyping import NDArray
from typing import List, Any, Tuple, Optional, Union, Set, Callable
from heapq import heappush, heappop
import os
import math


class MapView(View):
    __world: NodePath
    __map: Union[VoxelMap, FlatMap]

    __displays: List[Any]
    __persistent_displays: List[Any]
    __tracked_data: List[Any]

    __cube_modified: NDArray[(Any, Any, Any), bool]
    __cube_update_displays: List[MapDisplay]
    __previous_cube_update_displays: List[MapDisplay]
    __previous_cube_update_displays_rendered: List[bool]
    __cubes_requiring_update: Set[Point]
    __cube_colour: Colour

    __scratch: NodePath
    __overlay: NodePath

    __set_cube_colour: Callable[[Point, Colour, Colour], None]

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self.__displays = []
        self.__tracked_data = []

        self.__cube_update_displays = []
        self.__previous_cube_update_displays = []
        self.__previous_cube_update_displays_rendered = []
        self.__cubes_requiring_update = set()
        self.__cube_colour = None

        # world (dummy node)
        self.__world = self._services.graphics.window.render.attach_new_node("world")

        # MAP #
        extended_walls: bool = False
        map_size = self._services.algorithm.map.size
        map_data = np.empty((*map_size, 1) if map_size.n_dim == 2 else map_size, dtype=np.uint8)
        self.__cube_modified = np.empty(map_data.shape, dtype=bool)
        for x, y, z in np.ndindex(map_data.shape):
            p = Point(x, y) if map_size.n_dim == 2 else Point(x, y, z)
            i = self._services.algorithm.map.at(p)
            if i == Map.WALL_ID:
                map_data[x, y, z] = MapData.OBSTACLE_MASK
                self.__cube_modified[x, y, z] = False
            elif i == Map.EXTENDED_WALL_ID:
                map_data[x, y, z] = MapData.EXTENDED_WALL_MASK | MapData.TRAVERSABLE_MASK
                self.__cube_modified[x, y, z] = True
                extended_walls = True
            elif i == Map.UNMAPPED_ID:
                map_data[x, y, z] = MapData.UNMAPPED_MASK
                self.__cube_modified[x, y, z] = False
            else:
                map_data[x, y, z] = MapData.TRAVERSABLE_MASK
                self.__cube_modified[x, y, z] = True

        if map_size.n_dim == 2:
            self.__map = FlatMap(self._services, map_data, self.world)
            self.__set_cube_colour = lambda p, c, wfc: self.map.render_square(p, c, wfc)
        else:
            self.__map = VoxelMap(self._services, map_data, self.world)
            self.__set_cube_colour = lambda p, c, *discard: self.map.traversables_mesh.set_cube_colour(p, c)
        self.__map.center()

        self.__overlay = self.map.root.attach_new_node("overlay")
        self.__scratch = self.map.root.attach_new_node("scratch")
        self.renderer.push_root(self.__scratch)

        self.__entities_map_display = None
        self.__weight_grid_display = None
        self.__extended_walls_display = None

        self.__persistent_displays = [EntitiesMapDisplay(self._services)]
        self.__entities_map_display = self.__persistent_displays[-1]

        if extended_walls:
            grid = TrackedGrid(self.map.data, copy=False)
            dc = self._services.state.views.add_colour("extended wall", Colour(0.5).with_a(0.5))
            self.__persistent_displays.append(SolidGridMapDisplay(self._services, grid, dc, z_index=0, comparator=lambda x: bool(x & MapData.EXTENDED_WALL_MASK)))
            self.__extended_walls_display = self.__persistent_displays[-1]

        if hasattr(self._services.algorithm.map, "weight_grid"):
            mp = self._services.algorithm.map
            wg = TrackedGrid(mp.weight_grid, copy=False)
            dc_min = self._services.state.views.add_colour("min occupancy", BLACK.with_a(0))
            dc_max = self._services.state.views.add_colour("max occupancy", BLACK)
            display = GradientGridMapDisplay(self._services, wg, min_colour=dc_min, max_colour=dc_max, value_bounds=(0.1, mp.traversable_threshold))
            self.__persistent_displays.append(display)
            self.__weight_grid_display = self.__persistent_displays[-1]

        self.__deduced_traversables_colour = self._services.state.views.effective_view.colours[MapData.TRAVERSABLES]()
        self.__deduced_traversables_wf_colour = self._services.state.views.effective_view.colours[MapData.TRAVERSABLES_WF]()

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

        self.__update_view(True)
        self._services.ev_manager.broadcast(StateInitialisedEvent())

    def destroy(self) -> None:
        self.__map.destroy()
        self.__world.remove_node()
        self.__displays = []
        self.__persistent_displays = []
        self.__entities_map_display = None
        self.__weight_grid_display = None
        self.__extended_walls_display = None
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
            # when converting to/from SparseMap/DenseMap, require remapping
            self.__entities_map_display._map = self._services.algorithm.map

            if event.refresh:
                self.__refresh()
            self.__update_view(event.refresh)
        elif isinstance(event, MapUpdateEvent):
            self.__map_update(event.updated_cells)
        elif isinstance(event, ColourUpdateEvent):
            if event.view.is_effective():
                self.__update_view(False)
        elif isinstance(event, TakeMapScreenshotEvent):
            self.__take_hd_screenshot()

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

    def __update_cube_colour(self, p):
        if not bool(self.map.data[p.values] & MapData.TRAVERSABLE_MASK):
            return  # we don't allow colouring obstacles / unmapped, but requesting update is still valid

        self.__cube_colour = self.__deduced_traversables_colour
        for d in self.__cube_update_displays:
            d.update_cube(p)
        self.__set_cube_colour(p, self.__cube_colour, self.__deduced_traversables_wf_colour)
        self.__cube_modified[p.values] = self.__cube_colour != self.__deduced_traversables_colour

    def __map_update(self, updated_cells: List[Point]) -> None:
        self.__get_displays()
        while len(self.__displays) > 0:
            d = heappop(self.__displays)
            if d.updates_cubes:
                self.__cube_update_displays.append(d)

        if self.map.dim == 2:
            obs_c = self.map.obstacles_dc()
            obs_wfc = self.map.obstacles_wf_dc()

            for p in updated_cells:
                i = self._services.algorithm.map.at(p)
                if i == Map.WALL_ID:
                    self.map.data[p.values] = MapData.OBSTACLE_MASK
                    self.__cube_modified[p.values] = False
                    self.__set_cube_colour(p, obs_c, obs_wfc)
                elif i == Map.UNMAPPED_ID:
                    self.map.data[p.values] = MapData.UNMAPPED_MASK
                    self.__cube_modified[p.values] = False
                    self.__set_cube_colour(p, TRANSPARENT, TRANSPARENT)
                else:
                    self.map.data[p.values] = MapData.TRAVERSABLE_MASK

                    wgd = self.__weight_grid_display
                    if wgd is not None:
                        wgd.cube_colours[p.values] = wgd.get_colour(wgd.grid[p.values])

                    self.__cubes_requiring_update.add(p)
        else:  # 3D
            for p in updated_cells:
                i = self._services.algorithm.map.at(p)
                if i == Map.WALL_ID:
                    self.map.data[p.values] = MapData.OBSTACLE_MASK
                    self.__cube_modified[p.values] = False
                elif i == Map.UNMAPPED_ID:
                    self.map.data[p.values] = MapData.UNMAPPED_MASK
                    self.__cube_modified[p.values] = False
                else:
                    self.map.data[p.values] = MapData.TRAVERSABLE_MASK

            self.map.obstacles_mesh.structural_update(updated_cells)
            self.map.traversables_mesh.structural_update(updated_cells)

            for p in updated_cells:
                if bool(self.map.data[p.values] & MapData.TRAVERSABLE_MASK):
                    wgd = self.__weight_grid_display
                    if wgd is not None:
                        wgd.cube_colours[p.values] = wgd.get_colour(wgd.grid[p.values])

                    self.__cubes_requiring_update.add(p)

        self.__entities_map_display.render()

        for p in self.__cubes_requiring_update:
            self.__update_cube_colour(self.to_point3(p))

        self.__cube_update_displays.clear()
        self.__cubes_requiring_update.clear()

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
            if d.updates_cubes:
                if d._previous_cube_update_displays_idx is None:
                    d._previous_cube_update_displays_idx = len(self.__previous_cube_update_displays)
                    self.__previous_cube_update_displays_rendered.append(True)
                    self.__previous_cube_update_displays.append(d)
                else:
                    self.__previous_cube_update_displays_rendered[d._previous_cube_update_displays_idx] = True

        for d in self.__persistent_displays:
            add_diplay(d)
        for d in self._services.algorithm.instance.get_display_info():
            add_diplay(d)
        
        offset = 0
        for i in range(len(self.__previous_cube_update_displays)):
            i -= offset
            if not self.__previous_cube_update_displays_rendered[i]:
                self.__previous_cube_update_displays[i].request_update_all_cubes()
                self.__previous_cube_update_displays.pop(i)
                self.__previous_cube_update_displays_rendered.pop(i)
                offset += 1
                for j in range(i, len(self.__previous_cube_update_displays)):
                    self.__previous_cube_update_displays[j]._previous_cube_update_displays_idx = j

            else:
                self.__previous_cube_update_displays_rendered[i] = False

    def __render_displays(self, refresh: bool) -> None:
        while len(self.__displays) > 0:
            d: MapDisplay
            d = heappop(self.__displays)
            if d.updates_cubes:
                self.__cube_update_displays.append(d)
            d.render(refresh)
                

    def __update_cubes(self, refresh: bool) -> None:
        def init_eager_refresh():
            nonlocal refresh
            refresh = True
            for p in np.ndindex(self.map.data.shape):
                self.__cube_modified[p] = bool(self.map.data[p] & MapData.TRAVERSABLE_MASK)

        c = self._services.state.views.effective_view.colours[MapData.TRAVERSABLES]()
        if c != self.__deduced_traversables_colour:
            init_eager_refresh()
        self.__deduced_traversables_colour = c

        if self.map.dim == 2:
            wfc = self.map.traversables_wf_dc()
            wfc = self._services.state.views.effective_view.colours[MapData.TRAVERSABLES_WF]()
            if wfc != self.__deduced_traversables_wf_colour:
                init_eager_refresh()
            self.__deduced_traversables_wf_colour = wfc

        if refresh:
            for idx in np.ndindex(self.__cube_modified.shape):
                if self.__cube_modified[idx]:
                    p = Point(*idx)
                    self.__update_cube_colour(p)
                    lp = self.to_logical_point(p)

                    # we don't want to track extended walls once rendered.
                    # Note, they will still be refreshed when traversable
                    # (bg & wireframe) colour changes.
                    if bool(self.map.data[p.values] & MapData.EXTENDED_WALL_MASK):
                        self.__cube_modified[p.values] = False
                        self.__cubes_requiring_update.discard(p)

                    # we don't want to track occupancy grid weights
                    # Note, they will still be refreshed when traversable
                    # (bg & wireframe) colour changes.
                    if self.__weight_grid_display is not None and \
                       self._services.algorithm.map.at(lp) == Map.CLEAR_ID:
                        clr = self.__weight_grid_display.get_colour(self._services.algorithm.map.weight_grid[lp.values])

                        if (self.__deduced_traversables_colour == self.__cube_colour) or \
                           (clr is not None and blend_colours(clr, self.__deduced_traversables_colour) == self.__cube_colour):
                            self.__cube_modified[p.values] = False
                            self.__cubes_requiring_update.discard(p)

        # update these cubes regardless of refresh
        # since the cubes that require update aren't
        # necessarily modified.
        for p in self.__cubes_requiring_update:
            self.__update_cube_colour(p)

        for td in self.__tracked_data:
            td.clear_tracking_data()

        self.__tracked_data.clear()
        self.__cubes_requiring_update.clear()
        self.__cube_update_displays.clear()
        self.__displays.clear()

    def __update_view(self, refresh: bool) -> None:
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

    def __take_hd_screenshot(self):
        mx, my, mz = self.__map.data.shape

        # find the optimal zoom fit
        max_width = max(mx, my)
        max_height = max(mx, mz)
        tex = Texture()
        width = 4096
        height = 4096
        ss_buf = self._services.graphics.window.win.make_texture_buffer('hd_screenshot_buff', width, height, tex, True)

        cam = Camera('hd_cam')
        cam.set_lens(self._services.graphics.window.camLens.make_copy())
        cam.get_lens().set_aspect_ratio(width / height)
        np_cam = NodePath(cam)
        np_cam.reparent_to(self.__world)
        x, y, z = self.__world.get_pos()

        if mz > 1:
            # 3D map
            np_cam.set_pos(x - mx * 2.1, y - max_height * 3.6, z)
            np_cam.set_h(-30)
        else:
            # 2D map
            np_cam.set_pos(x, y, z + max_width * 2 + 1)
            np_cam.set_p(-90)

        ss_cam = self._services.graphics.window.make_camera(ss_buf, useCamera=np_cam)
        ss_scene = self._services.graphics.window.render
        ss_cam.node().set_scene(ss_scene)
        self._services.graphics.window.graphicsEngine.render_frame()
        tex = ss_buf.get_texture()
        ss_buf.set_active(False)

        self._services.resources.screenshots_dir.append(lambda fn: tex.write(fn))
        self._services.graphics.window.graphicsEngine.remove_window(ss_buf)

    def cube_center(self, p: Point) -> Point:
        x, y, z = self.to_point3(p)
        return Point(x + 0.5, y + 0.5, z - 0.5 if self.map.dim == 3 else z + 0.15)

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

    def draw_arc(self, p: Point, *args, **kwargs) -> None:
        self.renderer.draw_arc(self.cube_center(p), *args, **kwargs)

    def draw_circle(self, p: Point, *args, **kwargs) -> None:
        self.draw_arc(p, 2 * math.pi, *args, **kwargs)

    def draw_circle_filled(self, p: Point, *args, **kwargs) -> None:
        self.renderer.draw_circle_filled(self.cube_center(p), *args, **kwargs, radius=self.__circle_filled_radius)

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
