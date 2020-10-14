from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenImage import OnscreenImage

from typing import Tuple
import os, math

def make_arc(angle_degs = 360, nsteps = 16, thickness = 2, colour = (1,1,1)):
    ls = LineSegs()
    ls.set_thickness(thickness)
    ls.set_color(*colour)

    angle_rads = deg2Rad(angle_degs)

    for i in range(nsteps + 1):
        a = angle_rads * i / nsteps
        y = math.sin(a)
        x = math.cos(a)

        ls.drawTo(x, 0, y)

    node = ls.create()
    return NodePath(node)


class ViewStyle:
    base: ShowBase

    __palette_img: PNMImage
    __palette: OnscreenImage
    __palette_scale_factor: float
    __palette_size: Tuple[float, float] # (width, height)

    __picker: NodePath # circular sprite over picked region

    def __init__(self, base: ShowBase):
        self.base = base
        
        # PALETTE #
        palette_filename = os.path.join(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))), "data"), "colour_palette.png")
        self.__palette_img = PNMImage(Filename.fromOsSpecific(palette_filename))
        self.__palette = OnscreenImage(image=palette_filename)
        self.__palette.reparentTo(self.base.pixel2d)

        self.__palette_scale_factor = 1. / 10.
        self.__palette_size = (self.__palette_img.getReadXSize() * self.__palette_scale_factor, self.__palette_img.getReadYSize() * self.__palette_scale_factor)
        
        # default size of an OnscreenImage is 2x2 panda units (hence we must scale by half the actual size)
        self.__palette.set_scale(self.__palette_size[0] * .5, 1., self.__palette_size[1] * .5)

        # note that the origin of an OnscreenImage is at its center
        self.__palette.set_pos(self.base.win.get_size()[0] / 2, 0, -self.base.win.get_size()[1] / 2)

        self.base.accept("mouse1", self.__pick_colour)

        # PICKER #        
        core_thickness = 3.0
        border_thickness = 1.0
        border_colour = (0,0,0)

        self.__picker = make_arc(thickness=3.0)
        self.__picker.reparent_to(self.base.pixel2d)
        self.__picker.set_scale(5.0, 1., 5.0)

        # outer border
        border = make_arc(thickness=border_thickness, colour=border_colour)
        border.reparent_to(self.__picker)
        scale = (core_thickness + border_thickness) / core_thickness
        border.set_scale(scale, 1., scale)

        # inner border
        border = make_arc(thickness=border_thickness, colour=border_colour)
        border.reparent_to(self.__picker)
        scale = (core_thickness - 1) / core_thickness
        border.set_scale(scale, 1., scale)

        self.__picker.set_pos(self.__palette.get_pos())

    def __colour_at(self, pos: Tuple[float, float]):
        x, y = pos
        palette_x, _, palette_z = self.__palette.get_pos()
        
        rel_x = x - palette_x + self.__palette_size[0] / 2.
        rel_y = y + palette_z + self.__palette_size[1] / 2.

        if 0 <= rel_x < self.__palette_size[0] and 0 <= rel_y < self.__palette_size[1]:
            return self.__palette_img.getXel(int(rel_x / self.__palette_scale_factor), int(rel_y / self.__palette_scale_factor))
        else:
            return None

    def __colour_under_picker(self):
        x, _, z = self.__picker.get_pos()
        return self.__colour_at((x, -z))

    def __colour_under_mouse(self):
        pointer = self.base.win.get_pointer(0)
        return self.__colour_at((pointer.getX(), pointer.getY()))

    def __pick_colour(self):
        col = self.__colour_under_mouse()
        if col != None:
            print("Colour: " + str(col))
            pointer = self.base.win.get_pointer(0)
            x, y = pointer.getX(), pointer.getY()
            self.__picker.set_pos((x, 0, -y))

if __name__ == "__main__":
    app = ShowBase()
    vs = ViewStyle(app)
    app.run()