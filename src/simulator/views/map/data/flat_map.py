from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, Texture, PTA_uchar

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.square_mesh import SquareMesh
from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

import random
from typing import Any, Final

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

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "flat_map", square_size: int = 16):
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
        w = self.obstacles_data.shape[0] * square_size
        h = self.obstacles_data.shape[1] * square_size
        self.texture = Texture(self.name + "_texture")
        self.texture.setup_2d_texture(w, h, Texture.T_unsigned_byte, Texture.F_rgba8)
        self.square.set_texture(self.texture)
        self.texture.set_wrap_u(Texture.WM_clamp)
        self.texture.set_wrap_v(Texture.WM_clamp)
        self.texture.set_clear_color(LVecBase4f(*WHITE))

        self.render_obstacles()

    @property
    def dim(self) -> int:
        return 2

    def render_square(self, p: Point, c: Colour, wfc: Colour) -> None:
        tex_raw: PTA_uchar = self.texture.modifyRamImage()
        w = self.obstacles_data.shape[0] * self.square_size
        for y in range(p[1]*self.square_size, (p[1]+1)*self.square_size):
            for x in range(p[0]*self.square_size, (p[0]+1)*self.square_size):
                i = (x + y*w) * 4
                def conv(f): return int(round(f * 255))
                tex_raw[i + 0] = conv(c.b)  # B
                tex_raw[i + 1] = conv(c.g)  # G
                tex_raw[i + 2] = conv(c.r)  # R
                tex_raw[i + 3] = conv(c.a)  # A

    def render_obstacles(self) -> None:
        c = self.obstacles_dc()
        wfc = self.obstacles_wf_dc()

        for x, y, z in np.ndindex(self.obstacles_data.shape):
            if self.obstacles_data[x, y, z]:
                self.render_square((x, y, z), c, wfc)

    def destroy(self) -> None:
        super().destroy()
