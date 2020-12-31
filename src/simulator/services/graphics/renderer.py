from panda3d.core import NodePath, GeomNode, Geom, LineSegs, TextNode
from direct.showutil import BuildGeometry

from typing import Optional, List, Tuple
import math

from structures import Colour, Point, WHITE

class Renderer():
    __roots: List[NodePath]
    __draw_nps: List[NodePath]
    __circles: List[Tuple[int, float, Geom]]
    __thicknesses: List[float]
    __line_segs: LineSegs

    def __init__(self, root: NodePath) -> None:
        self.__roots = [root]
        self.__draw_nps = []
        self.__circles = []
        self.__thicknesses = [2.5]
        self.__line_segs = LineSegs()
        self.__line_segs.set_thickness(2.5)

    def push_root(self, np: NodePath) -> None:
        assert isinstance(np, NodePath)
        self.render()
        self.__roots.append(np)

    def pop_root(self) -> NodePath:
        self.render()
        return self.__roots.pop()

    def push_line_thickness(self, thickness: float) -> None:
        self.__render_lines()
        self.__thicknesses.append(thickness)
        self.__line_segs.set_thickness(thickness)

    def pop_line_thickness(self) -> float:
        self.__render_lines()
        self.__line_segs.set_thickness(self.__thicknesses[-2])
        return self.__thicknesses.pop()

    @property
    def line_segs(self) -> LineSegs:
        return self.__line_segs

    @property
    def root(self) -> NodePath:
        return self.__roots[-1]

    def render(self) -> None:
        self.__render_lines()

    def __render_lines(self) -> None:
        if not self.__line_segs.is_empty():
            n = self.__line_segs.create()
            self.root.attach_new_node(n)

    def draw_line(self, colour: Colour, p1: Point, p2: Point) -> None:
        ls = self.__line_segs
        ls.set_color(*colour)
        ls.move_to(*p1)
        ls.draw_to(*p2)

    def draw_sphere(self, p: Point, colour: Colour = WHITE, scale: float = 0.2) -> None:
        np = loader.load_model("models/misc/sphere.egg")
        np.set_color(*colour)
        np.reparent_to(self.root)
        np.set_pos(*p)
        np.set_scale(scale)

    def draw_arc(self, p: Point, angle_rads: float = 2 * math.pi, nsteps: int = 16, radius: float = 0.06, colour: Colour = WHITE) -> None:
        ls = self.__line_segs
        ls.set_color(*colour)

        x, y, z = p

        ls.move_to(x + radius, y, z)
        for i in range(nsteps + 1):
            a = angle_rads * i / nsteps
            ty = math.sin(a) * radius + y
            tx = math.cos(a) * radius + x
            ls.draw_to(tx, ty, z)

    def draw_circle(self, p: Point, *args, **kwargs) -> None:
        self.draw_arc(p, 2 * math.pi, *args, **kwargs)

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

        np = self.root.attach_new_node(gn)

        np.set_pos(*p)
        np.set_color(*colour)
