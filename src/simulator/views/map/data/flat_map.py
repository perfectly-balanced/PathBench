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

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "flat_map", square_size: int = 16):
        super().__init__(services, data, parent, name)

        self._add_colour(MapData.TRAVERSABLES)
        self._add_colour(MapData.TRAVERSABLES_WF)
        self._add_colour(MapData.OBSTACLES, callback=lambda dc: self.render_obstacles())
        self._add_colour(MapData.OBSTACLES_WF, callback=lambda dc: self.render_obstacles())

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

        self.square_mesh = SquareMesh()
        self.square = self.root.attach_new_node(self.square_mesh.geom_node)

        w = self.obstacles_data.shape[0] * square_size
        h = self.obstacles_data.shape[1] * square_size
        self.texture = Texture(self.name + "_texture")
        self.texture.setup_2d_texture(w, h, Texture.T_unsigned_byte, Texture.F_rgba8)
        self.square.set_texture(self.texture)
        self.texture.set_wrap_u(Texture.WM_clamp)
        self.texture.set_wrap_v(Texture.WM_clamp)
        self.texture.set_clear_color(LVecBase4f(*WHITE))

        tex_raw: PTA_uchar = self.texture.modifyRamImage()
        for y in range(2):
            for x in range(w):
                i = (x + y*w) * 4
                def conv(f): return round(f * 255)
                c = Colour(0.5, 0.2, 0.5, 1)
                tex_raw[i + 0] = conv(c.b)  # B
                tex_raw[i + 1] = conv(c.g)  # G
                tex_raw[i + 2] = conv(c.r)  # R
                tex_raw[i + 3] = conv(c.a)  # A

    def render_wf(self, p: Point) -> None:
        pass

    def render_obstacles(self) -> None:
        pass

    def destroy(self) -> None:
        super().destroy()
