from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *

from typing import Tuple, Union, Callable
import os
import math

from ..common import Colour

WINDOW_BG_COLOUR = Colour(.3, .3, .3, 1.)

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
    pick_colour_callback: Callable[[Tuple[float, float, float, float]], None]

    __base: ShowBase

    __palette_img: PNMImage
    __palette_size: Tuple[int, int]
    __palette_frame: DirectFrame

    __marker: DirectFrame
    __marker_center: DirectFrame

    def __init__(self, base: ShowBase, pick_colour_callback: Callable[[Tuple[float, float, float, float]], None], **kwargs) -> None:
        self.__base = base
        self.pick_colour_callback = pick_colour_callback

        # PALETTE #
        palette_filename = os.path.join(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))), "data"), "colour_palette.png")
        self.__palette_img = PNMImage(Filename.fromOsSpecific(palette_filename))
        self.__palette_size = (self.__palette_img.getReadXSize(), self.__palette_img.getReadYSize())
        self.__palette_frame = DirectFrame(image=palette_filename, **kwargs)
        self.__palette_frame['state'] = DGG.NORMAL
        self.__palette_frame.bind(DGG.B1PRESS, command=self.__pick)

        # MARKER #
        self.__marker = DirectFrame(parent=self.__palette_frame,
                                    frameColor=(0.0, 0.0, 0.0, 1.0),
                                    frameSize=(-0.08, .08, -.08, .08),
                                    pos=(0.0, 0.0, 0.0))

        self.__marker_center = DirectFrame(parent=self.__marker,
                                           frameSize=(-0.03, 0.03, -0.03, 0.03))
        
        self.__update_marker_colour()

    def __colour_at(self, x: float, y: float) -> Union[Tuple[float, float, float, float], None]:
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
            return (*self.__palette_img.getXel(int(x), int(y)), 1.0)
        else:
            return None
        
    def __update_marker_colour(self) -> Tuple[float, float, float, float]:
        c = self.colour_under_marker()
        if c is None:
            c = self.__marker_center['frameColor']
        else:
            self.__marker_center['frameColor'] = c
        return c

    def __update_marker_pos(self) -> None:
        if not self.__base.mouseWatcherNode.hasMouse():
            return None

        pointer = self.__base.win.get_pointer(0)
        x, y = pointer.getX(), -pointer.getY()

        w, h = self.__palette_size
        screen = self.__base.pixel2d

        img_scale = self.__palette_frame['image_scale']
        sx = self.__palette_frame.getSx(screen) * img_scale[0]
        sy = self.__palette_frame.getSz(screen) * img_scale[2]

        x -= self.__palette_frame.getX(screen)
        y -= self.__palette_frame.getZ(screen)
        x /= sx
        y /= sy

        x = max(-0.92, min(0.92, x))
        y = max(-0.92, min(0.92, y))

        self.__marker.set_pos(x, 0.0, y)

    def colour_under_marker(self) -> Union[Tuple[float, float, float, float], None]:
        x, _, y = self.__marker.get_pos()

        w, h = self.__palette_size
        screen = self.__base.pixel2d
        
        img_scale = self.__palette_frame['image_scale']
        sx = self.__palette_frame.getSx(screen) * img_scale[0]
        sy = self.__palette_frame.getSz(screen) * img_scale[2]

        x *= sx
        y *= sy
        x += self.__palette_frame.getX(screen)
        y += self.__palette_frame.getZ(screen)

        return self.__colour_at(x, y)

    def colour_under_mouse(self) -> Union[Tuple[float, float, float, float], None]:
        if not self.__base.mouseWatcherNode.hasMouse():
            return None

        pointer = self.__base.win.get_pointer(0)
        return self.__colour_at(pointer.getX(), -pointer.getY())

    def __pick(self, *args):
        self.__update_marker_pos()
        self.pick_colour_callback(self.__update_marker_colour())

    @property
    def frame(self) -> DirectFrame:
        return self.__palette_frame

class Window():
    __base: ShowBase
    __name: str
    
    __frame: DirectFrame
    __drag_start: Point2
    __drag_offset: Vec2

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

class ColourView():
    def __init__(self, parent: DirectFrame, colour: Union[Colour, None] = None):
        self.__frame = DirectFrame(parent=parent,
                                  relief=DGG.SUNKEN,
                                  borderWidth=(.05, .05),
                                  frameColor=WINDOW_BG_COLOUR,
                                  frameSize=(-.4, .4, -.3, .3),
                                  scale=(.5, 1., .5))
        self.__view = DirectFrame(parent=self.__frame,
                                    frameColor=colour if colour else WINDOW_BG_COLOUR,
                                    frameSize=(-.35, .35, -.25, .25))
        self.__colour = colour

    @property
    def colour(self) -> str:
        return 'colour'
    
    @colour.getter
    def colour(self) -> Union[Colour, None]:
        return self.__colour

    @colour.setter
    def colour(self, value) -> None:
        self.__colour = value
        self.__view['frameColor'] = value if value else WINDOW_BG_COLOUR
    
    @property
    def frame(self) -> DirectFrame:
        return self.__frame

    def set_colour(self, value) -> None:
        self.colour = value

class ViewEditor():
    __base: ShowBase
    __window = Window
    __colour_picker: ColourPicker

    def __init__(self, base: ShowBase):
        self.__base = base

        self.__window = Window(self.__base, "view_editor", parent=self.__base.pixel2d,
                                            relief=DGG.RAISED,
                                            borderWidth=(0.0, 0.0),
                                            frameColor=(.5, .5, .5, 1.),
                                            pos=(150, 0., -200),
                                            scale=(150, 1., 150),
                                            frameSize = (-0.8, 0.8, -2.0, 1.0))

        self.__colour_picker = ColourPicker(self.__base,
                                            lambda col: print("Colour: " + str(col)),
                                            parent=self.__window.frame,
                                            relief=DGG.SUNKEN,
                                            borderWidth=(.0, .0),
                                            image_scale=(1., 1., 1.),
                                            frameColor=WINDOW_BG_COLOUR,
                                            frameSize=(-1., 1., -1., 1.),
                                            scale=(0.75, 1., 0.75))
    
        self.__cv_picked = ColourView(self.__window.frame)
        self.__cv_picked.frame.set_pos(-0.55, 0.0, -1.3)
        self.__cv_hovered = ColourView(self.__window.frame)
        self.__cv_hovered.frame.set_pos(-0.55, 0.0, -0.95)
        self.__colour_picker.pick_colour_callback = self.__cv_picked.set_colour

        def update_cv_hovered(task):
            c = self.__colour_picker.colour_under_mouse()
            self.__cv_hovered.set_colour(c)
            return task.cont

        self.__update_cv_hovered_tk = self.__base.taskMgr.add(update_cv_hovered, 'update_cv_hovered')