from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, Texture, PTA_uchar

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.square_mesh import SquareMesh
from simulator.views.util import blend_colours
from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

import array
import random
import math
from typing import Any, Final, Tuple, Dict, Optional

import numpy as np
from nptyping import NDArray

class FlatMap(MapData):
    square: Final[NodePath]
    square_mesh: Final[SquareMesh]
    texture: Final[Texture]

    square_size: Final[int]
    wf_thickness: Final[float]
    wf_line_cnt: Final[int]

    traversables_dc: Final[DynamicColour]
    traversables_wf_dc: Final[DynamicColour]
    obstacles_dc: Final[DynamicColour]
    obstacles_wf_dc: Final[DynamicColour]

    logical_w: Final[int]
    logical_h: Final[int]
    texture_w: Final[int]
    texture_h: Final[int]

    __lines: Dict[Tuple[Colour, Colour], Tuple[bytes, bytes, bytes]]

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "flat_map", square_size: Optional[int] = None, wf_thickness: Optional[float] = None):
        super().__init__(services, data, parent, name)

        self.__lines = {}

        self.logical_w = int(self.obstacles_data.shape[0])
        self.logical_h = int(self.obstacles_data.shape[1])

        if square_size is None:
            # basic heuristic for decreasing square
            # resolution as map increases size.
            square_size = 32
            while (self.logical_w * square_size) > 4400:
                square_size //= 2
                if square_size == 4:
                    break
            while (self.logical_h * square_size) > 4400:
                square_size //= 2
                if square_size == 4:
                    break
                
        self.square_size = square_size
        self.texture_w = int(self.logical_w * square_size)
        self.texture_h = int(self.logical_h * square_size)

        if wf_thickness is None:
            # basic heuristic for decreasing wireframe
            # thickness as square size decreases.
            wf_thickness = 4
            while (square_size / wf_thickness) < 16:
                wf_thickness /= 2

        self.wf_thickness = wf_thickness
        self.wf_line_cnt = int(max(1, math.ceil(self.wf_thickness + 1) // 2))

        self.traversables_dc = self._add_colour(MapData.TRAVERSABLES)
        self.traversables_wf_dc = self._add_colour(MapData.TRAVERSABLES_WF)
        self.obstacles_dc = self._add_colour(MapData.OBSTACLES, invoke_callback=False, callback=lambda dc: self.render_obstacles())
        self.obstacles_wf_dc = self._add_colour(MapData.OBSTACLES_WF, invoke_callback=False, callback=lambda dc: self.render_obstacles())

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

        self.square_mesh = SquareMesh(self.obstacles_data.shape[0], self.obstacles_data.shape[1])
        self.square = self.root.attach_new_node(self.square_mesh.geom_node)

        self.texture = Texture(self.name + "_texture")
        self.texture.setup_2d_texture(self.texture_w, self.texture_h, Texture.T_unsigned_byte, Texture.F_rgba8)
        self.square.set_texture(self.texture)
        self.texture.set_wrap_u(Texture.WM_clamp)
        self.texture.set_wrap_v(Texture.WM_clamp)
        self.texture.set_clear_color(LVecBase4f(*WHITE))

        self.render_obstacles()

    @property
    def dim(self) -> int:
        return 2

    def __create_lines(self, c: Colour, wfc: Colour) -> Tuple[bytes, bytes, bytes]:
        def conv(f): return int(round(f * 255))
        def to_pixel(c): return (conv(c.b), conv(c.g), conv(c.r), conv(c.a))

        wfd2 = int(self.wf_thickness // 2)

        if wfd2 != self.wf_line_cnt:
            bc = blend_colours(wfc.with_a(wfc.a * ((self.wf_thickness / 2) - wfd2)), c)
            tpixel = to_pixel(bc)
        else:
            bc = None
        
        pixel = to_pixel(c)
        wfc = blend_colours(wfc, c)
        wfpixel = to_pixel(wfc)

        line = array.array('B')
        for _ in range(wfd2):
            line.extend(wfpixel)
        if bc is not None:
            line.extend(tpixel)
        for _ in range(self.square_size - 2 * self.wf_line_cnt):
            line.extend(pixel)
        if bc is not None:
            line.extend(tpixel)
        for _ in range(wfd2):
            line.extend(wfpixel)
        lbytes = line.tobytes()

        wfline = array.array('B')
        for _ in range(self.square_size):
            wfline.extend(wfpixel)
        wflbytes = wfline.tobytes()

        if bc is None:
            tlbytes = wflbytes
        else:
            tline = array.array('B')
            for _ in range(wfd2):
                tline.extend(wfpixel)
            tline.extend(tpixel)
            for _ in range(self.square_size - 2 * self.wf_line_cnt):
                tline.extend(tpixel)
            tline.extend(tpixel)
            for _ in range(wfd2):
                tline.extend(wfpixel)
            tlbytes = tline.tobytes()

        self.__lines[(c, wfc)] = (lbytes, wflbytes, tlbytes)
        return lbytes, wflbytes, tlbytes

    def render_square(self, p: Point, c: Colour, wfc: Colour) -> None:
        px, py = int(p[0]), int(p[1])
        l, wfl, tl = self.__lines[(c, wfc)] if (c, wfc) in self.__lines else self.__create_lines(c, wfc)

        img = self.texture.modify_ram_image()
        x_offset = px * self.square_size
        lsize = self.square_size * 4
        for y in range(py * self.square_size, py * self.square_size + self.wf_line_cnt - 1):
            img.set_subdata((y * self.texture_w + x_offset) * 4, lsize, wfl)
        img.set_subdata(((py * self.square_size + self.wf_line_cnt - 1) * self.texture_w + x_offset) * 4, lsize, tl)
        for y in range(py * self.square_size + self.wf_line_cnt, (py + 1) * self.square_size - self.wf_line_cnt):
            img.set_subdata((y * self.texture_w + x_offset) * 4, lsize, l)
        img.set_subdata((((py + 1) * self.square_size - self.wf_line_cnt) * self.texture_w + x_offset) * 4, lsize, tl)
        for y in range((py + 1) * self.square_size - self.wf_line_cnt + 1, (py + 1) * self.square_size):
            img.set_subdata((y * self.texture_w + x_offset) * 4, lsize, wfl)

    def render_obstacles(self) -> None:
        c = self.obstacles_dc()
        wfc = self.obstacles_wf_dc()

        for x, y, z in np.ndindex(self.obstacles_data.shape):
            if self.obstacles_data[x, y, z]:
                self.render_square((x, y, z), c, wfc)

    def destroy(self) -> None:
        super().destroy()
