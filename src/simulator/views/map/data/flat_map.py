from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, Texture, SamplerState

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.square_mesh import SquareMesh
from simulator.views.util import blend_colours
from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK
from utility.compatibility import Final

import array
import random
import math
from typing import Any, Tuple, Dict, Optional, List
from numbers import Real

import numpy as np
from nptyping import NDArray

class FlatMap(MapData):
    squares: Final[List[NodePath]]
    square_meshes: Final[List[SquareMesh]]
    textures: Final[List[Texture]]

    square_size: Final[int]
    wf_thickness: Final[float]
    wf_line_cnt: Final[int]

    traversables_dc: Final[DynamicColour]
    traversables_wf_dc: Final[DynamicColour]
    obstacles_dc: Final[DynamicColour]
    obstacles_wf_dc: Final[DynamicColour]

    texture_w: Final[int]
    texture_h: Final[int]

    __lines: Dict[Tuple[Colour, Colour], Tuple[bytes, bytes, bytes]]

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "flat_map", square_size: Optional[int] = None, wf_thickness: Optional[float] = None, block_size: Optional[int] = 200, depth: Real = 0.1):
        super().__init__(services, data, parent, name)

        self.__lines = {}

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

        if block_size is None:
            block_size = max(self.logical_w, self.logical_h)
        self.block_size = block_size
        self.num_blocks_x = (self.logical_w // self.block_size) + int(self.logical_w % self.block_size != 0)
        self.num_blocks_y = (self.logical_h // self.block_size) + int(self.logical_h % self.block_size != 0)

        self.texture_w = int(self.block_size * square_size)
        self.texture_h = self.texture_w

        if wf_thickness is None:
            # basic heuristic for decreasing wireframe
            # thickness as square size decreases.
            wf_thickness = 4
            while (square_size / wf_thickness) < 16:
                wf_thickness /= 2

        self.wf_thickness = wf_thickness
        self.wf_line_cnt = int(max(1, math.ceil(self.wf_thickness + 1) // 2))

        self.depth = depth

        self.traversables_dc = self._add_colour(MapData.TRAVERSABLES)
        self.traversables_wf_dc = self._add_colour(MapData.TRAVERSABLES_WF)
        self.obstacles_dc = self._add_colour(MapData.OBSTACLES, invoke_callback=False, callback=lambda dc: self.render_obstacles())
        self.obstacles_wf_dc = self._add_colour(MapData.OBSTACLES_WF, invoke_callback=False, callback=lambda dc: self.render_obstacles())

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

        self.squares = [None for _ in range(self.num_blocks_x * self.num_blocks_y)]
        self.square_meshes = self.squares.copy()
        self.textures = self.squares.copy()
        for y in range(self.num_blocks_y):
            for x in range(self.num_blocks_x):
                idx = y * self.num_blocks_x + x
                
                # find a no_tex_coord that would make unseen squares transparent
                no_tex_coord = None
                for py in range(y * self.num_blocks_y, (y+1) * self.num_blocks_y):
                    for px in range(x * self.num_blocks_x, (x+1) * self.num_blocks_x):
                        if py >= self.logical_h or px >= self.logical_w:
                            no_tex_coord = ((px % self.num_blocks_x) / self.num_blocks_x, (py % self.num_blocks_y) / self.num_blocks_y)
                            break
                    if no_tex_coord is not None:
                        break
                if no_tex_coord is None:
                    no_tex_coord = (1.0, 1.0)

                mesh = self.square_meshes[idx] = SquareMesh(self.block_size, self.block_size, self.depth, no_tex_coord)
                np = self.squares[idx] = self.root.attach_new_node(mesh.geom_node)
                np.set_pos((x * self.block_size, y * self.block_size, 0))

                tex = self.textures[idx] = Texture(self.name + "_texture")
                tex.setup_2d_texture(self.texture_w, self.texture_h, Texture.T_unsigned_byte, Texture.F_rgba8)
                np.set_texture(tex)
                tex.set_wrap_u(Texture.WM_clamp)
                tex.set_wrap_v(Texture.WM_clamp)
                # tex.set_clear_color(LVecBase4f(*WHITE))
                tex.set_border_color(LVecBase4f(*TRANSPARENT))

        for y in range(self.num_blocks_y * self.block_size):
            for x in range(self.num_blocks_x * self.block_size):
                if y >= self.logical_h or x >= self.logical_w:
                    self.render_square((x, y), TRANSPARENT, TRANSPARENT)

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
        
        # find corresponding block texture & offset points accordingly
        block_x_idx = px // self.block_size
        block_y_idx = py // self.block_size
        img = self.textures[block_y_idx * self.num_blocks_x + block_x_idx].modify_ram_image()
        px = px % self.block_size
        py = py % self.block_size

        x_offset = px * self.square_size
        lsize = self.square_size * 4

        # bottom wireframe lines
        for y in range(py * self.square_size, py * self.square_size + self.wf_line_cnt - 1):
            img.set_subdata((y * self.texture_w + x_offset) * 4, lsize, wfl)

        # transition line between wireframe and body
        img.set_subdata(((py * self.square_size + self.wf_line_cnt - 1) * self.texture_w + x_offset) * 4, lsize, tl)

        # body lines
        for y in range(py * self.square_size + self.wf_line_cnt, (py + 1) * self.square_size - self.wf_line_cnt):
            img.set_subdata((y * self.texture_w + x_offset) * 4, lsize, l)

        # transition line between wireframe and body
        img.set_subdata((((py + 1) * self.square_size - self.wf_line_cnt) * self.texture_w + x_offset) * 4, lsize, tl)

        # top wireframe lines
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
