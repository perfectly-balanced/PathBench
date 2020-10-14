from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *

from typing import Tuple, Union, Callable
import os
import math

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


class ColourPicker:
    pick_colour_callback: Callable[[LRGBColorf], None]

    __base: ShowBase

    __palette_img: PNMImage
    __palette_size: Tuple[int, int]
    __palette_frame: DirectFrame

    __picker: NodePath # circular sprite over picked region

    def __init__(self, base: ShowBase, pick_colour_callback: Callable[[LRGBColorf], None], **kwargs) -> None:
        self.__base = base
        self.pick_colour_callback = pick_colour_callback

        # PALETTE #
        palette_filename = os.path.join(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))), "data"), "colour_palette.png")
        self.__palette_img = PNMImage(Filename.fromOsSpecific(palette_filename))
        self.__palette_size = (self.__palette_img.getReadXSize(), self.__palette_img.getReadYSize())
        self.__palette_frame = DirectFrame(image=palette_filename, **kwargs)
        self.__palette_frame['state'] = DGG.NORMAL
        self.__palette_frame.bind(DGG.B1PRESS, command=self.__pick_colour)

        # PICKER #        
        core_thickness = 3.0
        border_thickness = 1.0
        border_colour = (0,0,0)

        self.__picker = make_arc(thickness=3.0)
        self.__picker.reparent_to(self.__base.pixel2d)
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

        self.__picker.set_pos((self.__palette_frame.getX(self.__base.pixel2d), 0, self.__palette_frame.getZ(self.__base.pixel2d)))

    def __colour_at(self, x: float, y: float) -> Union[LRGBColorf, None]:
        w, h = self.__palette_size
        screen = self.__base.pixel2d

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
        if not self.__base.mouseWatcherNode.hasMouse():
            return None

        pointer = self.__base.win.get_pointer(0)
        return self.__colour_at(pointer.getX(), -pointer.getY())

    def __pick_colour(self, *args):
        print("HERE")
        colour = self.__colour_under_mouse()
        if colour != None:
            self.pick_colour_callback(colour)
            pointer = self.__base.win.get_pointer(0)
            x, y = pointer.getX(), pointer.getY()
            self.__picker.set_pos((x, 0, -y))

class Window():
    __base: ShowBase
    __name: str
    
    __frame: DirectFrame
    __drag_start: Point2()
    __drag_offset: Vec2()

    def __init__(self, base: ShowBase, name: str, *args, **kwargs):
        self.__base = base
        self.__name = name

        if 'frameSize' not in kwargs:
            kwargs['frameSize'] = (-.8, .8, -1., 1.)

        self.__frame = DirectFrame(*args, **kwargs)
        self.__frame['state'] = DGG.NORMAL
        self.__frame.bind(DGG.B1PRESS, command=self.__start_drag)
        self.__frame.bind(DGG.B1RELEASE, command=self.__stop_drag)

        self.__drag_start = Point2()
        self.__drag_offset = Vec2()

    def __drag(self, task):
        if not self.__base.mouseWatcherNode.hasMouse():
            return task.cont

        pointer = self.__base.win.getPointer(0)
        pos = Point2(pointer.getX(), -pointer.getY())
        x, z = pos + self.__drag_offset
        self.__frame.setPos(x, 0., z)
        self.__drag_start = Point2(pos)

        return task.cont

    def __start_drag(self, *args):
        pointer = self.__base.win.getPointer(0)
        fx, _, fz = self.__frame.getPos()

        self.__drag_start = Point2(pointer.getX(), -pointer.getY())
        self.__drag_offset = Point2(fx, fz) - self.__drag_start

        self.__base.taskMgr.add(self.__drag, 'drag_' + self.__name)

    def __stop_drag(self, *args):
        self.__base.taskMgr.remove('drag_' + self.__name)
    
    @property
    def frame(self) -> DirectFrame:
        return self.__frame

class ViewEditor():
    __base: ShowBase
    __window = Window
    __colour_picker: ColourPicker

    def __init__(self, base: ShowBase):
        self.__base = base

        self.__window = Window(self.__base, "view_editor", parent=self.__base.pixel2d,
                                            relief=DGG.RAISED,
                                            borderWidth=(.05, .05),
                                            frameColor=(.5, .5, .5, 1.),
                                            pos=(150, 0., -200),
                                            scale=(150, 1., 150))

        self.__colour_picker = ColourPicker(self.__base,
                                            lambda col: print("Colour: " + str(col)),
                                            parent=self.__window.frame,
                                            relief=DGG.SUNKEN,
                                            borderWidth=(.05, .05),
                                            image_scale=(.95, 1., .95),
                                            frameColor=(.3, .3, .3, 1.),
                                            frameSize=(-1., 1., -1., 1.),
                                            pos=(-.15, 0., .3),
                                            scale=(.5, 1., .5))

if __name__ == "__main__":
    app = ShowBase()
    vs = ViewEditor(app)
    app.run()