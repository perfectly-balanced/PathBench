from panda3d.core import *
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase

import os
import math
import json
from typing import Tuple, Union, Callable, List

from structures import Colour, WHITE, BLACK, TRANSPARENT
from constants import DATA_PATH

from simulator.services.services import Services
from simulator.views.map.data.voxel_map import VoxelMap

from simulator.views.gui.common import WINDOW_BG_COLOUR, WIDGET_BG_COLOUR

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
        palette_filename = os.path.join(DATA_PATH, "colour_palette.png")
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
        bg_filename = os.path.join(DATA_PATH, "colour_bg.png")

        self.__frame = DirectFrame(parent=parent,
                                  relief=DGG.SUNKEN,
                                  image=bg_filename,
                                  image_scale=(0.465, 1.0, 0.39),
                                  borderWidth=(0.05, 0.05),
                                  frameSize=(-0.52, 0.52, -0.44, 0.44),
                                  scale=(0.5, 1.0, 0.5))
        self.__view = DirectFrame(parent=self.__frame,
                                    frameColor=colour if colour != None else TRANSPARENT,
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
                                    text_fg=WHITE,
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
                                   text_fg=WHITE,
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


class AdvancedColourPicker():
    __base: ShowBase
    __colour: Colour
    __callback: Callable[[Colour], None]

    __frame: DirectFrame
    __colour_picker: ColourPicker

    def __init__(self, base: ShowBase, parent: DirectFrame, callback: Callable[[Colour], None], colour: Colour = Colour(0.25, 0.5, 0.75, 1.0)):
        self.__base = base
        self.__colour = colour
        self.__callback = callback

        self.__frame = DirectFrame(parent=parent)
        
        self.__colour_picker = ColourPicker(self.__base,
                                            self.__colour_picked_callback,
                                            parent=self.__frame,
                                            relief=DGG.SUNKEN,
                                            borderWidth=(.0, .0),
                                            image_scale=(1., 1., 1.),
                                            frameColor=WIDGET_BG_COLOUR,
                                            frameSize=(-1., 1., -1., 1.),
                                            scale=(1.0, 1.0, 0.75),
                                            pos=(0.0, 0.0, 0.15))
    
        self.__cv_picked = ColourView(self.__frame, self.__colour)
        self.__cv_picked.frame.set_pos(-0.74, 0.0, -1.41)
        self.__cv_hovered = ColourView(self.__frame)
        self.__cv_hovered.frame.set_pos(-0.74, 0.0, -0.89)

        def update_cv_hovered(task):
            c = self.__colour_picker.colour_under_mouse()
            self.__cv_hovered.colour = c
            return task.cont
        self.__base.taskMgr.add(update_cv_hovered, 'update_cv_hovered')

        c = self.__colour
        f = self.__frame
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

        self.__dirty_rem_no_hide = 0

    def __colour_picked_callback(self, colour: Colour):
        n = 3
        r, g, b, _ = colour
        self.__r.update(r)
        self.__g.update(g)
        self.__b.update(b)
        self.__cv_picked.colour = Colour(r, g, b, self.__cv_picked.colour.a)
        self.__dirty_rem_no_hide = 3 # something fishy is going on
        self.__callback(self.__cv_picked.colour)

    def __colour_channel_slider_edit_callback(self, chn: ColourChannel, value: float):
        if chn != self.__a:
            if self.__dirty_rem_no_hide:
                self.__dirty_rem_no_hide -= 1
            else:
                self.__colour_picker.marker.hide()
        
        chn.update_entry(value)
        self.__cv_picked.colour = Colour(self.__r.value, self.__g.value, self.__b.value, self.__a.value)
        self.__callback(self.__cv_picked.colour)

    def __colour_channel_entry_edit_callback(self, chn: ColourChannel, value: float):
        if chn != self.__a:
            if self.__dirty_rem_no_hide:
                self.__dirty_rem_no_hide -= 1
            else:
                self.__colour_picker.marker.hide()
        
        chn.update_slider(value)
        self.__cv_picked.colour = Colour(self.__r.value, self.__g.value, self.__b.value, self.__a.value)
        self.__callback(self.__cv_picked.colour)
    
    @property
    def frame(self) -> DirectFrame:
        return self.__frame
    
    @property
    def colour(self) -> str:
        return 'colour'
    
    @colour.getter
    def colour(self) -> Colour:
        return self.self.__cv_picked.colour
    
    @colour.setter
    def colour(self, value: Colour) -> None:
        self.__colour_picker.marker.hide()
        self.__cv_picked.colour = value
        r, g, b, a = value
        self.__r.update(r)
        self.__g.update(g)
        self.__b.update(b)
        self.__a.update(a)

class ViewElement():
    __name: str
    __visibility_callback: Callable[[str, bool], None]
    __colour_callback: Callable[[str, Colour], None]
    __visible: bool

    __frame: DirectFrame
    __label: DirectLabel
    __cv: ColourView
    __visibility_btn: DirectFrame

    def __init__(self, name: str, visibility_callback: Callable[[str, bool], None], colour_callback: Callable[[str, Colour], None], parent: DirectFrame, visible: bool = True, colour: Colour = Colour(0.2, 0.3, 0.4, 0.5)):
        self.__name = name
        self.__visible = visible
        self.__visibility_callback = None
        self.__colour_callback = None

        self.__frame = DirectFrame(parent=parent, frameColor=WINDOW_BG_COLOUR)

        self.__cv = ColourView(self.__frame, colour)
        self.__cv.frame.set_scale((0.15, 1.0, 0.15))
        self.__cv.frame.set_pos((-0.65, 1.0, 0.0))
        
        self.__label = DirectLabel(parent=self.__frame,
                                   text=self.__name,
                                   text_fg=WHITE,
                                   text_bg=WINDOW_BG_COLOUR,
                                   borderWidth=(.0, .0),
                                   pos=(0.28, 0.0, -0.03),
                                   scale=(0.1, 1.0, 0.1))

        visibility_filename = os.path.join(DATA_PATH, "visible.png")
        self.__visibility_btn = DirectButton(parent=self.__frame,
                                             image=visibility_filename,
                                             frameColor=TRANSPARENT,
                                             pos=(-0.92, 0.0, 0.0),
                                             scale=(0.09, 1.0, 0.06),
                                             command=self.__toggle_visible)
                                            
        self.__visibility_bar = DirectFrame(parent=self.__frame,
                                            borderWidth=(.0, .0),
                                            frameColor=WINDOW_BG_COLOUR,
                                            frameSize=(-0.1, 0.1, -0.01, 0.01),
                                            pos=(-0.92, 0.0, 0.0),
                                            hpr=(40, 0, 40))
        
        self.visible = self.__visible # trigger UI update

        self.__visibility_callback = visibility_callback
        self.__colour_callback = colour_callback

    def __toggle_visible(self):
        self.visible = not self.visible

    @property
    def name(self) -> str:
        return self.__name

    @property
    def frame(self) -> DirectFrame:
        return self.__frame

    @property
    def colour_view(self) -> ColourView:
        return self.__cv

    @property
    def colour(self) -> str:
        return 'colour'
    
    @colour.getter
    def colour(self) -> Colour:
        return self.__cv.colour
    
    @colour.setter
    def colour(self, value: Colour) -> None:
        self.__cv.colour = value
        if self.__colour_callback:
            self.__colour_callback(self.name, value)
    
    @property
    def visible(self) -> str:
        return 'visible'
    
    @visible.getter
    def visible(self) -> bool:
        return self.__visible
    
    @visible.setter
    def visible(self, value: bool) -> None:
        self.__visible = value
        if self.__visibility_callback:
            self.__visibility_callback(self.name, value)
        if self.__visible:
            self.__visibility_bar.hide()
        else:
            self.__visibility_bar.show()

class ViewEditor():
    __services: Services
    __base: ShowBase
    __window: Window
    __colour_picker: AdvancedColourPicker
    __elements: List[ViewElement]
    __voxel_map: VoxelMap

    def __init__(self, services: Services, voxel_map: VoxelMap):
        self.__services = services
        self.__base = self.__services.window
        self.__voxel_map = voxel_map
        self.hidden = False

        self.state_num = 0
        self.__window = Window(self.__base, "view_editor", parent=self.__base.pixel2d,
                               relief=DGG.RAISED,
                               borderWidth=(0.0, 0.0),
                               frameColor=WINDOW_BG_COLOUR,
                               pos=(1900, 200, -350),
                               scale=(150, 1., 150),
                               frameSize=(-1.1, 1.1, -5.82, 1.56))

        self.__colour_picker = AdvancedColourPicker(self.__base, self.__window.frame, self.__colour_picker_callback)

        # spacers
        DirectFrame(parent=self.__window.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1., 1., -0.01, 0.01),
                    pos=(0.0, 0.0, -1.75))

        DirectFrame(parent=self.__window.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1., 1., -0.01, 0.01),
                    pos=(0.0, 0.0, 1.1))

        DirectFrame(parent=self.__window.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1., 1., -0.01, 0.01),
                    pos=(0.0, 0.0, -4.1))
        
        # selected colour view frame
        self.__selected_cv_outline = DirectFrame(parent=self.__window.frame,
                                                relief=DGG.SUNKEN,
                                                frameColor=WHITE,
                                                borderWidth=(0.15, 0.15),
                                                frameSize=(-0.62, 0.62, -0.54, 0.54),
                                                scale=(0.18, 1.0, 0.18))
        # selected state number frame
        self.__selected_state_number = DirectFrame(parent=self.__window.frame,
                                                 relief=DGG.SUNKEN,
                                                 frameColor=WHITE,
                                                 borderWidth=(0.15, 0.15),
                                                 frameSize=(-0.62, 0.62, -0.54, 0.54),
                                                 scale=(0.3, 2.0, 0.35))

        self.heading = DirectLabel(parent=self.__window.frame,
                                   text="View Editor",
                                   text_fg=WHITE,
                                   text_bg=WINDOW_BG_COLOUR,
                                   borderWidth=(.0, .0),
                                   #pos=(10.9, 1.4, 3),
                                   scale=(0.2, 3, 0.2))

        self.heading.set_pos((0.0, 0.0, 1.27))


        quit_filename = os.path.join(DATA_PATH, "quit.png")

        # Quit button
        self.btn = DirectButton(image=quit_filename,
                                command=self.__toggle_view_editor,
                                pos=(0.9, 0.4, 1.32),
                                parent=self.__window.frame,
                                scale=0.1,
                                pressEffect=1,
                                frameColor=TRANSPARENT)

        # Show or hide the Style Editor #
        self.__base.accept('v', self.__toggle_view_editor)

        self.__save_outline = DirectFrame(parent=self.__window.frame,
                                                 frameColor=WHITE,
                                                 pos=(-0.57, 0, -5.45),
                                                 borderWidth=(0.25, 0.15),
                                                 frameSize=(-0.62, 0.62, -0.54, 0.54),
                                                 scale=(0.50, 2.1, 0.25))

        self.__quit_outline = DirectFrame(parent=self.__window.frame,
                                          frameColor=WHITE,
                                          pos=(0.50, 0, -5.45),
                                          borderWidth=(0.25, 0.15),
                                          frameSize=(-0.62, 0.62, -0.54, 0.54),
                                          scale=(0.65, 2.1, 0.25))

        # save and restore
        self.btn_s = DirectButton(
                                 text = "Save",
                                 text_fg=(0.3, 0.3, 0.3, 1.0),
                                 pressEffect = 1,
                                 command=lambda: self.save_state(self.state_num),
                                 pos=(-0.575, 0, -5.5),
                                 parent=self.__window.frame,
                                 scale=(0.20, 2.1, 0.15),
                                 frameColor=TRANSPARENT)
        
        self.btn_r = DirectButton(
                                 text="Restore",
                                 text_fg=(0.3, 0.3, 0.3, 1.0),
                                 pressEffect=1,
                                 command=lambda: self.load_state(self.state_num),
                                 pos=(0.50, 0, -5.5),
                                 parent=self.__window.frame,
                                 scale=(0.20, 2.1, 0.15),
                                 frameColor=TRANSPARENT)

        # view elements
        self.__elements = []
        for name, dc in self.__voxel_map.colours.items():
            self.__elements.append(ViewElement(name, self.__voxel_map.set_visibility, self.__voxel_map.set_colour, self.__window.frame, colour=dc()))

        for i in range(len(self.__elements)):
            self.__elements[i].frame.set_pos((0.0, 0.0, -1.9 - 0.2 * i))
        
        self.__cv_transparent_overlays = []
        for i in range(len(self.__elements)):
            self.__cv_transparent_overlays.append(DirectButton(parent=self.__window.frame,
                                                relief=DGG.SUNKEN,
                                                frameColor=TRANSPARENT,
                                                borderWidth=(0, 0),
                                                frameSize=(-0.52, 0.52, -0.44, 0.44),
                                                scale=(0.18, 1.0, 0.18),
                                                pos=(-0.65, 1.0, -1.897 - 0.2 * i),
                                                command=self.__select_cv,
                                                extraArgs=[i]))

        # Creating state buttons
        self.__state_btns = []
        for i in range (0, 3):
            num = os.path.join(DATA_PATH, str(i + 1) + ".png")

            self.__state_btns.append(DirectButton(image=num,
                                 pos=(-0.7 + i * 0.7, 0.4, -4.4),
                                 parent=self.__window.frame,
                                 scale=0.16,
                                 frameColor=TRANSPARENT,
                                 command=self.__select_state_num_one,
                                 extraArgs=[i]))
        for i in range (0, 3):
            num = os.path.join(DATA_PATH, str(i + 4) + ".png")
            self.__state_btns.append(DirectButton(image=num,
                                 pos=(-0.7 + i * 0.7, 0.4, -4.9),
                                 parent=self.__window.frame,
                                 scale=0.16,
                                 frameColor=TRANSPARENT,
                                 command = self.__select_state_num_two,
                                 extraArgs=[i+3]))

        """
        # default settings - testing #
        self.OBSTACLES = self.__elements[0]
        self.OBSTACLES_TRI_WF = self.__elements[1]
        self.OBSTACLES_SQR_WF = self.__elements[2]
        self.TRAVERSABLES = self.__elements[3]
        self.TRAVERSABLES_TRI_WF = self.__elements[4]
        self.TRAVERSABLES_SQR_WF = self.__elements[5]
        self.START = self.__elements[6]
        self.GOAL = self.__elements[7]
        self.PATH = self.__elements[8]
        self.FRINGE = self.__elements[9]
        self.EXPLORED = self.__elements[10]
        """

        self.views = [{'OBSTACLES': [0.9254902601242065, 0.0, 1.0, 0.5], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],'TRAVERSABLES': [1.0, 1.0, 1.0, 0.0],
        'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
        'START': [0.8, 0.0, 0.0, 1.0], 'GOAL': [0.0, 0.9, 0.0, 1.0], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]},

                 {'OBSTACLES': [0.8, 0.2, 0.2, 1.0], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES': [1.0, 1.0, 1.0, 0.5],
                  'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
                  'START': [0.5, 0.0, 0.5, 1.0], 'GOAL': [0.0, 1.0, 0.0, 0.5], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]},

                 {'OBSTACLES': [0.8, 0.2, 0.2, 1.0], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES': [1.0, 1.0, 1.0, 0.5],
                  'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
                  'START': [0.5, 0.0, 0.5, 1.0], 'GOAL': [0.0, 1.0, 0.0, 0.5], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]},

                 {'OBSTACLES': [0.8, 0.2, 0.2, 1.0], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES': [1.0, 1.0, 1.0, 0.5],
                  'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
                  'START': [0.5, 0.0, 0.5, 1.0], 'GOAL': [0.0, 1.0, 0.0, 0.5], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]},

                 {'OBSTACLES': [0.8, 0.2, 0.2, 1.0], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES': [1.0, 1.0, 1.0, 0.5],
                  'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
                  'START': [0.5, 0.0, 0.5, 1.0], 'GOAL': [0.0, 1.0, 0.0, 0.5], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]},

                 {'OBSTACLES': [0.8, 0.2, 0.2, 1.0], 'OBSTACLES_TRI_WF': [1.0, 1.0, 1.0, 1.0],
                  'OBSTACLES_SQR_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES': [1.0, 1.0, 1.0, 0.5],
                  'TRAVERSABLES_TRI_WF': [1.0, 1.0, 1.0, 0.5], 'TRAVERSABLES_SQR_WF': [1.0, 1.0, 1.0, 0.5],
                  'START': [0.5, 0.0, 0.5, 1.0], 'GOAL': [0.0, 1.0, 0.0, 0.5], 'PATH': [1.0, 1.0, 1.0, 0.5],
                  'FRINGE': [0.2, 0.2, 0.8, 1.0], 'EXPLORED': [0.4, 0.4, 0.4, 1.0]}
                 ]

        if os.path.isfile('state.json'):
            with open('state.json', 'r') as json_file:
                self.views = json.load(json_file)
                # todo: actually check that structure is the same
        else:
            self.save_state()

        self.__selected_cv_idx = 0
        self.__select_cv(0)
        self.__selected_sn_idx = 0
        self.__select_state_num_one(0)

    def load_state(self, i : int):
        state_nr = self.views[i]
        """
        self.OBSTACLES.colour = Colour(state_nr["OBSTACLES"][0], state_nr["OBSTACLES"][1], state_nr["OBSTACLES"][2], state_nr["OBSTACLES"][3])
        self.OBSTACLES_TRI_WF.colour = Colour(state_nr["OBSTACLES_TRI_WF"][0], state_nr["OBSTACLES_TRI_WF"][1], state_nr["OBSTACLES_TRI_WF"][2], state_nr["OBSTACLES_TRI_WF"][3])
        self.OBSTACLES_SQR_WF.colour = Colour(state_nr["OBSTACLES_SQR_WF"][0], state_nr["OBSTACLES_SQR_WF"][1], state_nr["OBSTACLES_SQR_WF"][2], state_nr["OBSTACLES_SQR_WF"][3])
        self.TRAVERSABLES.colour = Colour(state_nr["TRAVERSABLES"][0], state_nr["TRAVERSABLES"][1], state_nr["TRAVERSABLES"][2], state_nr["TRAVERSABLES"][3])
        self.TRAVERSABLES_TRI_WF.colour = Colour(state_nr["TRAVERSABLES_TRI_WF"][0], state_nr["TRAVERSABLES_TRI_WF"][1], state_nr["TRAVERSABLES_TRI_WF"][2], state_nr["TRAVERSABLES_TRI_WF"][3])
        self.TRAVERSABLES_SQR_WF.colour = Colour(state_nr["TRAVERSABLES_SQR_WF"][0], state_nr["TRAVERSABLES_SQR_WF"][1], state_nr["TRAVERSABLES_SQR_WF"][2], state_nr["TRAVERSABLES_SQR_WF"][3])
        self.START.colour = Colour(state_nr["START"][0], state_nr["START"][1], state_nr["START"][2], state_nr["START"][3])
        self.GOAL.colour = Colour(state_nr["GOAL"][0], state_nr["GOAL"][1], state_nr["GOAL"][2], state_nr["GOAL"][3])
        self.PATH.colour = Colour(state_nr["PATH"][0], state_nr["PATH"][1], state_nr["PATH"][2], state_nr["PATH"][3])
        self.FRINGE.colour = Colour(state_nr["FRINGE"][0], state_nr["FRINGE"][1], state_nr["FRINGE"][2], state_nr["FRINGE"][3])
        self.EXPLORED.colour = Colour(state_nr["EXPLORED"][0], state_nr["EXPLORED"][1], state_nr["EXPLORED"][2], state_nr["EXPLORED"][3])
        """

    def save_state(self, i = None):
        if i is not None:
            state_nr = self.views[i]
            """
            state_nr["OBSTACLES"][0],state_nr["OBSTACLES"][1],state_nr["OBSTACLES"][2],state_nr["OBSTACLES"][3] = self.OBSTACLES.colour
            state_nr["OBSTACLES_TRI_WF"][0], state_nr["OBSTACLES_TRI_WF"][1], state_nr["OBSTACLES_TRI_WF"][2], state_nr["OBSTACLES_TRI_WF"][3] = self.OBSTACLES_TRI_WF.colour
            state_nr["OBSTACLES_SQR_WF"][0], state_nr["OBSTACLES_SQR_WF"][1], state_nr["OBSTACLES_SQR_WF"][2], state_nr["OBSTACLES_SQR_WF"][3] = self.OBSTACLES_SQR_WF.colour
            state_nr["TRAVERSABLES"][0], state_nr["TRAVERSABLES"][1], state_nr["TRAVERSABLES"][2], state_nr["TRAVERSABLES"][3] = self.TRAVERSABLES.colour
            state_nr["TRAVERSABLES_TRI_WF"][0], state_nr["TRAVERSABLES_TRI_WF"][1], state_nr["TRAVERSABLES_TRI_WF"][2], state_nr["TRAVERSABLES_TRI_WF"][3] = self.TRAVERSABLES_TRI_WF.colour
            state_nr["TRAVERSABLES_SQR_WF"][0], state_nr["TRAVERSABLES_SQR_WF"][1], state_nr["TRAVERSABLES_SQR_WF"][2], state_nr["TRAVERSABLES_SQR_WF"][3] = self.OBSTACLES_SQR_WF.colour
            state_nr["START"][0], state_nr["START"][1], state_nr["START"][2], state_nr["START"][3] = self.START.colour
            state_nr["GOAL"][0], state_nr["GOAL"][1], state_nr["GOAL"][2], state_nr["GOAL"][3] = self.GOAL.colour
            state_nr["PATH"][0], state_nr["PATH"][1], state_nr["PATH"][2], state_nr["PATH"][3] = self.PATH.colour
            state_nr["FRINGE"][0], state_nr["FRINGE"][1], state_nr["FRINGE"][2], state_nr["FRINGE"][3] = self.FRINGE.colour
            state_nr["EXPLORED"][0], state_nr["EXPLORED"][1], state_nr["EXPLORED"][2], state_nr["EXPLORED"][3] = self.EXPLORED.colour
            """

        with open('state.json', 'w') as json_file:
            json.dump(self.views, json_file)

    def __toggle_view_editor(self):
        if not self.hidden:
            self.__window.frame.hide()
            self.hidden = True
        else:
            self.__window.frame.show()
            self.hidden = False


    def __colour_picker_callback(self, colour: Colour) -> None:
        i = self.__selected_cv_idx
        self.__elements[i].colour = colour

    def __select_cv(self, i: int):
        self.__selected_cv_idx = i
        self.__selected_cv_outline.set_pos((-0.65, 1.0, -1.897 - 0.2 * i))
        self.__colour_picker.colour = self.__elements[i].colour

    def __select_state_num_one(self, i: int):
        self.__selected_sn_idx = i
        self.__selected_state_number.set_pos((-0.7 + i * 0.7, 0.4, -4.4))
        self.load_state(i)
        self.state_num = i

    def __select_state_num_two(self, i: int):
        self.__selected_sn_idx = i
        self.__selected_state_number.set_pos((-0.7 + (i-3) * 0.7, 0.4, -4.9))
        self.load_state(i)
        self.state_num = i
