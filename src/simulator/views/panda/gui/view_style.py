from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *

from typing import Tuple, Union
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
    __palette_size: Tuple[int, int]
    __palette_frame: DirectFrame

    __picker: NodePath # circular sprite over picked region

    def __init__(self, base: ShowBase) -> None:
        self.base = base
        
        # PALETTE #
        palette_filename = os.path.join(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))), "data"), "colour_palette.png")
        self.__palette_img = PNMImage(Filename.fromOsSpecific(palette_filename))
        self.__palette_size = (self.__palette_img.getReadXSize(), self.__palette_img.getReadYSize())
        self.__palette_frame = DirectFrame(relief=DGG.SUNKEN,
                                           borderWidth=(.05, .05),
                                           image=palette_filename,
                                           image_scale=(.95, 1., .95),
                                           frameColor=(.3, .3, .3, 1.),
                                           frameSize=(-1., 1., -1., 1.),
                                           pos=(-.15, 0., .3),
                                           scale=(.25, 1., .25))

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

        self.__picker.set_pos((self.__palette_frame.getX(self.base.pixel2d), 0, self.__palette_frame.getZ(self.base.pixel2d)))

    def __colour_at(self, x: float, y: float) -> Union[LRGBColorf, None]:
        w, h = self.__palette_size
        screen = self.base.pixel2d

        img_scale = self.__palette_frame['image_scale']
        sx = self.__palette_frame.getSx(screen) * img_scale[0]
        sy = self.__palette_frame.getSz(screen) * img_scale[2]

        x -= self.__palette_frame.getX(screen)
        y -= self.__palette_frame.getZ(screen)
        x = (0.5 + x / (2.0 * sx)) * w
        y = (0.5 - y / (2.0 * sy)) * h

        if 0 <= x < w and 0 <= y < h:
            return self.__palette_img.getXel(int(x), int(y))
        else:
            return None

    def __colour_under_picker(self) -> Union[LRGBColorf, None]:
        x, _, z = self.__picker.get_pos()
        return self.__colour_at(x, z)

    def __colour_under_mouse(self) -> Union[LRGBColorf, None]:
        if not self.base.mouseWatcherNode.hasMouse():
            return None

        pointer = self.base.win.get_pointer(0)
        return self.__colour_at(pointer.getX(), -pointer.getY())

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