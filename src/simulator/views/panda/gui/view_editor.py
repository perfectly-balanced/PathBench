from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *

from typing import Tuple, Union, Callable
import os
import math

from ..common import Colour

WINDOW_BG_COLOUR = Colour(0.5, 0.5, 0.5, 1.0)
WIDGET_BG_COLOUR = Colour(0.3, 0.3, 0.3, 1.0)

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
        self.__marker.hide()

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
        self.__marker.show()

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
    
    @property
    def marker(self) -> DirectFrame:
        return self.__marker

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
    __frame: DirectFrame
    __view: DirectFrame
    __colour: Union[Colour, None]

    def __init__(self, parent: DirectFrame, colour: Union[Colour, None] = None):
        self.__frame = DirectFrame(parent=parent,
                                  relief=DGG.SUNKEN,
                                  borderWidth=(0.05, 0.05),
                                  frameColor=WIDGET_BG_COLOUR,
                                  frameSize=(-0.52, 0.52, -0.44, 0.44),
                                  scale=(0.5, 1.0, 0.5))
        self.__view = DirectFrame(parent=self.__frame,
                                    frameColor=colour if colour != None else WINDOW_BG_COLOUR,
                                    frameSize=(-0.47, 0.47, -0.39, 0.39))
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
        self.__view['frameColor'] = value if value != None else WINDOW_BG_COLOUR
    
    @property
    def frame(self) -> DirectFrame:
        return self.__frame

class ColourChannel():
    __slider_edit_callback: Callable[['ColourChannel', float], None]
    __entry_edit_callback: Callable[['ColourChannel', float], None]

    __frame: DirectFrame
    __label: DirectLabel
    __slider: DirectSlider
    __entry: DirectEntry

    def __init__(self, parent: DirectFrame, text: str, value: float, slider_edit_callback: Callable[['ColourChannel', float], None], entry_edit_callback: Callable[['ColourChannel', float], None]):
        self.__frame = DirectFrame(parent=parent)
        self.__slider_edit_callback = slider_edit_callback
        self.__entry_edit_callback = entry_edit_callback
        
        self.__label = DirectLabel(parent=self.__frame,
                                    text=text,
                                    text_fg=Colour(1.0),
                                    text_bg=WINDOW_BG_COLOUR,
                                    pos=(-0.5, 0.0, 0.0),
                                    scale=(0.1, 1.0, 0.1))
        
        self.__slider = DirectSlider(parent=self.__frame,
                                    orientation=DGG.HORIZONTAL,
                                    borderWidth=(0.0, 0.0),
                                    frameColor=WIDGET_BG_COLOUR,
                                    frameSize=(-1.0, 1.0, -0.4, 0.4),
                                    thumb_frameSize=(-0.075, 0.075, -0.2, 0.2),
                                    value=value,
                                    pos=(0.05, 0.0, 0.0255),
                                    scale=(0.45, 1.0, 0.5))

        self.__entry = DirectEntry(parent=self.__frame,
                                   frameColor=WIDGET_BG_COLOUR,
                                   text_fg=Colour(1.0),
                                   initialText=str(value),
                                   scale=0.1,
                                   width=3,
                                   pos=(0.55, 0.0, -0.01105))

        self.__set_callbacks()

    def __unset_callbacks(self):
        self.__slider['command'] = None
        self.__entry['command'] = None

    def __set_callbacks(self):
        self.__slider['command'] = lambda: self.__slider_edit_callback(self, self.__slider['value'])
        self.__entry['command'] = lambda s: self.__entry_edit_callback(self, float(s))

    @property
    def frame(self) -> DirectFrame:
        return self.__frame
    
    def update_slider(self, value: float):
        self.__unset_callbacks()
        self.__slider['value'] = value
        self.__set_callbacks()

    def update_entry(self, value: float):
        self.__unset_callbacks()
        self.__entry.enterText(f'{value:.3f}')
        self.__set_callbacks()

    def update(self, value: float):
        self.update_slider(value)
        self.update_entry(value)

    @property
    def value(self) -> float:
        return self.__slider['value']

class ViewEditor():
    __base: ShowBase
    __window = Window
    __colour_picker: ColourPicker
    __colour: Colour

    def __init__(self, base: ShowBase):
        self.__base = base
        self.__colour = Colour(0.25, 0.5, 0.75, 1.0)

        self.__window = Window(self.__base, "view_editor", parent=self.__base.pixel2d,
                                            relief=DGG.RAISED,
                                            borderWidth=(0.0, 0.0),
                                            frameColor=WINDOW_BG_COLOUR,
                                            pos=(150, 0., -200),
                                            scale=(150, 1., 150),
                                            frameSize = (-1.1, 1.1, -2.0, 1.0))

        self.__colour_picker = ColourPicker(self.__base,
                                            lambda col: print("Colour: " + str(col)),
                                            parent=self.__window.frame,
                                            relief=DGG.SUNKEN,
                                            borderWidth=(.0, .0),
                                            image_scale=(1., 1., 1.),
                                            frameColor=WIDGET_BG_COLOUR,
                                            frameSize=(-1., 1., -1., 1.),
                                            scale=(1.0, 1.0, 0.75),
                                            pos=(0.0, 0.0, 0.15))
    
        self.__cv_picked = ColourView(self.__window.frame, self.__colour)
        self.__cv_picked.frame.set_pos(-0.74, 0.0, -1.41)
        self.__cv_hovered = ColourView(self.__window.frame)
        self.__cv_hovered.frame.set_pos(-0.74, 0.0, -0.89)
        self.__colour_picker.pick_colour_callback = self.__colour_picked_callback

        def update_cv_hovered(task):
            c = self.__colour_picker.colour_under_mouse()
            self.__cv_hovered.colour = c
            return task.cont
        self.__base.taskMgr.add(update_cv_hovered, 'update_cv_hovered')

        c = self.__colour
        f = self.__window.frame
        sc = self.__colour_channel_slider_edit_callback
        ec = self.__colour_channel_entry_edit_callback
        self.__r = ColourChannel(f, "R", c[0], sc, ec)
        self.__g = ColourChannel(f, "G", c[1], sc, ec)
        self.__b = ColourChannel(f, "B", c[2], sc, ec)
        self.__a = ColourChannel(f, "A", c[3], sc, ec)

        x = 0.14
        y_base = -0.8
        y_inc = -0.25
        self.__r.frame.set_pos((x, 0.0, y_base))
        self.__g.frame.set_pos((x, 0.0, y_base+y_inc))
        self.__b.frame.set_pos((x, 0.0, y_base+y_inc*2))
        self.__a.frame.set_pos((x, 0.0, y_base+y_inc*3))

    def __colour_picked_callback(self, colour: Colour):
        n = 3
        r, g, b, _ = colour
        self.__r.update(r)
        self.__g.update(g)
        self.__b.update(b)
        self.__cv_picked.colour = Colour(r, g, b, self.__cv_picked.colour.a)

    def __colour_channel_slider_edit_callback(self, chn: ColourChannel, value: float):
        if chn != self.__a:
            self.__colour_picker.marker.hide()
        
        chn.update_entry(value)
        self.__cv_picked.colour = Colour(self.__r.value, self.__g.value, self.__b.value, self.__a.value)

    def __colour_channel_entry_edit_callback(self, chn: ColourChannel, value: float):
        if chn != self.__a:
            self.__colour_picker.marker.hide()
        
        chn.update_slider(value)
        self.__cv_picked.colour = Colour(self.__r.value, self.__g.value, self.__b.value, self.__a.value)