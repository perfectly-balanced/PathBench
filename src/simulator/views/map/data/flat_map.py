from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, Texture, SamplerState

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.geometry.meshes.square_mesh import SquareMesh
from simulator.views.util import blend_colours
from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK
from algorithms.configuration.entities.obstacle import Obstacle
from utility.compatibility import Final

import array
import random
import math
from typing import Any, Tuple, Dict, Optional, List
from numbers import Real

import numpy as np
from nptyping import NDArray

from lru import LRU

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

    def __init__(self,
                 services: Services,
                 data: NDArray[(Any, Any, Any), bool],
                 parent: NodePath,
                 name: str = "flat_map",
                 square_size: Optional[int] = None,
                 wf_thickness: Optional[float] = None,
                 block_size: Optional[int] = None,
                 depth: Real = 0.1):
        super().__init__(services, data, parent, name)

        self.__lines = LRU(16)

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
            # make block fit entire map
            block_size = max(self.logical_w, self.logical_h)

            # ensure block size is a power of 2 so that the
            # size of its texture is a power of 2.
            if not ((block_size & (block_size - 1) == 0) and block_size != 0):
                n = 1
                while n < block_size:
                    n = n << 1
                block_size = n
            
            # make sure block texture size is at most 2048x2048,
            # any higher than this would cause poor performance
            # on low-end graphics cards.
            while (block_size * self.square_size) > 2048:
                block_size = block_size >> 1
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

        init_tex_data = array.array('B')
        for _ in range(self.block_size * self.square_size * self.block_size * self.square_size):
            init_tex_data.append(0)
        self.__init_tex_data = init_tex_data.tobytes()

        self.squares = [None for _ in range(self.num_blocks_x * self.num_blocks_y)]
        self.square_meshes = self.squares.copy()
        self.textures = self.squares.copy()
        self.render_obstacles()

    @property
    def dim(self) -> int:
        return 2
    
    def __add_block(self, x: int, y: int) -> None:
        idx = y * self.num_blocks_x + x

        mesh = self.square_meshes[idx] = SquareMesh(self.block_size, self.block_size, self.depth)
        np = self.squares[idx] = self.root.attach_new_node(mesh.geom_node)
        np.set_pos((x * self.block_size, y * self.block_size, 0))

        tex = self.textures[idx] = Texture(self.name + "_texture")
        tex.setup_2d_texture(self.texture_w, self.texture_h, Texture.T_unsigned_byte, Texture.F_rgba8)
        tex.modify_ram_image().set_subdata(0, len(self.__init_tex_data), self.__init_tex_data)
        np.set_texture(tex)
        tex.set_wrap_u(Texture.WM_clamp)
        tex.set_wrap_v(Texture.WM_clamp)

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
        wfpixel = to_pixel(blend_colours(wfc, c))

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

        ls = self.__lines.get((c, wfc))
        l, wfl, tl = self.__create_lines(c, wfc) if ls is None else ls
        
        # find corresponding block texture & offset points accordingly
        block_x_idx = px // self.block_size
        block_y_idx = py // self.block_size
        tex = self.textures[block_y_idx * self.num_blocks_x + block_x_idx]
        if tex is None:
            if c == TRANSPARENT and wfc == TRANSPARENT:
                return
            self.__add_block(block_x_idx, block_y_idx)
            tex = self.textures[block_y_idx * self.num_blocks_x + block_x_idx]
        img = tex.modify_ram_image()
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

        for o in self._services.algorithm.map.obstacles:
            if type(o) is Obstacle:
                self.render_square(o.position, c, wfc)

    def center(self) -> None:
        world = self.root.get_parent()
        (_, _, z1), (_, _, z2) = self.root.get_tight_bounds()
        self.root.set_pos(-self.logical_w / 2,
                          -self.logical_h / 2,
                          world.getZ() - (z2 - z1) / 2)

    def destroy(self) -> None:
        super().destroy()
