from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, Texture, PTA_uchar

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.square_mesh import SquareMesh
from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

import random
from typing import Any, Final
import array

import numpy as np
from nptyping import NDArray

class FlatMap(MapData):
    square: Final[NodePath]
    square_mesh: Final[SquareMesh]
    texture: Final[Texture]
    square_size: Final[int]

    traversables_dc: Final[DynamicColour]
    traversables_wf_dc: Final[DynamicColour]
    obstacles_dc: Final[DynamicColour]
    obstacles_wf_dc: Final[DynamicColour]

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "flat_map", square_size: int = 32):
        super().__init__(services, data, parent, name)

        self.traversables_dc = self._add_colour(MapData.TRAVERSABLES)
        self.traversables_wf_dc = self._add_colour(MapData.TRAVERSABLES_WF)
        self.obstacles_dc = self._add_colour(MapData.OBSTACLES, invoke_callback=False, callback=lambda dc: self.render_obstacles())
        self.obstacles_wf_dc = self._add_colour(MapData.OBSTACLES_WF, invoke_callback=False, callback=lambda dc: self.render_obstacles())

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

        self.square_mesh = SquareMesh()
        self.square = self.root.attach_new_node(self.square_mesh.geom_node)

        self.square_size = square_size
        self.width = self.obstacles_data.shape[0] * square_size
        self.height = self.obstacles_data.shape[1] * square_size
        self.texture = Texture(self.name + "_texture")
        self.texture.setup_2d_texture(self.width, self.height, Texture.T_unsigned_byte, Texture.F_rgba8)
        self.square.set_texture(self.texture)
        self.texture.set_wrap_u(Texture.WM_clamp)
        self.texture.set_wrap_v(Texture.WM_clamp)
        self.texture.set_clear_color(LVecBase4f(*WHITE))

        self.render_obstacles()

    @property
    def dim(self) -> int:
        return 2

    def render_square(self, p: Point, c: Colour) -> None:
        def conv(f): return int(round(f * 255))
        r, g, b, a = conv(c.r), conv(c.g), conv(c.b), conv(c.a)
        px, py = p[0], p[1]

        line = array.array('B')
        pixel = (b, g, r, a)
        for _ in range(self.square_size):
            line.extend(pixel)
        line_str = line.tostring()

        img = self.texture.modifyRamImage()
        x_offset = px * self.square_size
        line_size = self.square_size * 4
        for y in range(py * self.square_size, (py + 1) * self.square_size):            
            img.set_subdata((y * self.width + x_offset) * 4, line_size, line_str)

    def blend(self, src: Colour, dst: Colour):
        wda = dst.a * (1 - src.a)  # weighted dst alpha
        a = src.a + wda
        d = (a if a != 0 else 1)
        r = (src.r * src.a + dst.r * wda) / d
        g = (src.g * src.a + dst.g * wda) / d
        b = (src.b * src.a + dst.b * wda) / d
        return Colour(r, g, b, a)

    def render_square_with_wf(self, p: Point, c: Colour, wfc: Colour) -> None:
        def conv(f): return int(round(f * 255))

        wfc = self.blend(wfc, c)
        px, py = p[0], p[1]
        r, g, b, a = conv(c.r), conv(c.g), conv(c.b), conv(c.a)
        wfr, wfg, wfb, wfa = conv(wfc.r), conv(wfc.g), conv(wfc.b), conv(wfc.a)

        WF_THICKNESS = 4
        HALF_WF_THICKNESS = WF_THICKNESS // 2

        pixel = (b, g, r, a)
        wfpixel = (wfb, wfg, wfr, wfa)

        line = array.array('B')
        for _ in range(HALF_WF_THICKNESS):
            line.extend(wfpixel)
        for _ in range(self.square_size - WF_THICKNESS):
            line.extend(pixel)
        for _ in range(HALF_WF_THICKNESS):
            line.extend(wfpixel)
        line_str = line.tostring()

        wfline = array.array('B')
        for _ in range(self.square_size):
            wfline.extend(wfpixel)
        wfline_str = wfline.tostring()

        img = self.texture.modifyRamImage()
        x_offset = px * self.square_size
        lsize = self.square_size * 4
        for y in range(py * self.square_size, py * self.square_size + HALF_WF_THICKNESS):            
            img.set_subdata((y * self.width + x_offset) * 4, lsize, wfline_str)
        for y in range(py * self.square_size + HALF_WF_THICKNESS, (py + 1) * self.square_size - HALF_WF_THICKNESS):            
            img.set_subdata((y * self.width + x_offset) * 4, lsize, line_str)
        for y in range((py + 1) * self.square_size - HALF_WF_THICKNESS, (py + 1) * self.square_size):
            img.set_subdata((y * self.width + x_offset) * 4, lsize, wfline_str)

    def render_obstacles(self) -> None:
        c = self.obstacles_dc()
        wfc = self.obstacles_wf_dc()

        for x, y, z in np.ndindex(self.obstacles_data.shape):
            if self.obstacles_data[x, y, z]:
                self.render_square_with_wf((x, y, z), c, wfc)

    def destroy(self) -> None:
        super().destroy()
