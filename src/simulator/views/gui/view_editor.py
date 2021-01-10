from panda3d.core import PNMImage, Filename, TextNode
from direct.gui.DirectGui import DirectFrame, DGG, DirectButton, DirectLabel, DirectSlider, DirectEntry, DirectScrolledFrame
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject

from typing import Tuple, Union, Callable, List
import os

from structures import Colour, WHITE, BLACK, TRANSPARENT
from utility.constants import GUI_DATA_PATH

from simulator.services.services import Services
from simulator.services.persistent_state.persistent_state_views import PersistentStateViews
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.new_colour_event import NewColourEvent
from simulator.services.event_manager.events.toggle_view_event import ToggleViewEvent

from simulator.views.gui.common import WINDOW_BG_COLOUR, WIDGET_BG_COLOUR
from simulator.views.gui.window import Window


class ColourPicker:
    pick_colour_callback: Callable[[Tuple[float, float, float, float]], None]

    __base: ShowBase

    __palette_img: PNMImage
    __palette_size: Tuple[int, int]
    __palette_frame: DirectFrame

    __marker: DirectFrame
    __marker_center: DirectFrame

    enabled: bool

    def __init__(self, base: ShowBase, pick_colour_callback: Callable[[Tuple[float, float, float, float]], None],
                 **kwargs) -> None:
        self.__base = base
        self.pick_colour_callback = pick_colour_callback
        self.enabled = True

        # PALETTE #
        palette_filename = os.path.join(GUI_DATA_PATH, "colour_palette.png")
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
        if self.enabled:
            self.__update_marker_pos()
            self.pick_colour_callback(self.__update_marker_colour())

    @property
    def frame(self) -> DirectFrame:
        return self.__palette_frame

    @property
    def marker(self) -> DirectFrame:
        return self.__marker


class ColourView():
    __frame: DirectFrame
    __view: DirectFrame
    __colour: Union[Colour, None]

    def __init__(self, parent: DirectFrame, colour: Union[Colour, None] = None):
        bg_filename = os.path.join(GUI_DATA_PATH, "colour_bg.png")

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

    @property
    def view(self) -> DirectFrame:
        return self.__view

    def destroy(self) -> None:
        self.__view.destroy()
        self.__frame.destroy()


class ColourChannel(DirectObject):
    __slider_edit_callback: Callable[['ColourChannel', float], None]
    __entry_edit_callback: Callable[['ColourChannel', float], None]

    __frame: DirectFrame
    __label: DirectLabel
    __slider: DirectSlider
    __entry: DirectEntry
    __disable_frame_overlay: DirectButton
    __enabled: bool

    def __init__(self, parent: DirectFrame, text: str, value: float,
                 slider_edit_callback: Callable[['ColourChannel', float], None],
                 entry_edit_callback: Callable[['ColourChannel', float], None],
                 mouse1_press_callbacks: List[Callable[[], None]]):
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

        self.__entry_hovered = False
        mouse1_press_callbacks.append(self.__entry_mouse_click_callback)
        self.__entry = DirectEntry(parent=self.__frame,
                                   frameColor=WIDGET_BG_COLOUR,
                                   text_fg=WHITE,
                                   initialText=str(value),
                                   scale=0.1,
                                   width=3,
                                   suppressKeys=True,
                                   pos=(0.55, 0.0, -0.01105))
        self.__entry.bind(DGG.EXIT, self.__entry_exit_callback)
        self.__entry.bind(DGG.ENTER, self.__entry_enter_callback)
        self.__entry.bind(DGG.B1PRESS, self.__entry_mouse_click_callback)
        self.accept("mouse1", self.__entry_mouse_click_callback)

        self.__disable_frame_overlay = DirectButton(parent=self.__frame,
                                                    frameColor=TRANSPARENT,
                                                    borderWidth=(0.0, 0.0),
                                                    frameSize=(-0.6, 0.9, -0.2, 0.2),
                                                    suppressMouse=True)
        self.__disable_frame_overlay.hide()
        self.__enabled = True

        self.__set_callbacks()

    def __entry_exit_callback(self, *discard) -> None:
        self.__entry_hovered = False

    def __entry_enter_callback(self, *discard) -> None:
        self.__entry_hovered = True

    def __entry_mouse_click_callback(self, *discard) -> None:
        if self.__entry_hovered:
            s = self.__entry.get()
            try:
                f = float(s)
            except:
                return
            self.__entry_edit_callback(self, f)
        else:
            self.__entry['focus'] = False

    def __unset_callbacks(self):
        self.__slider['command'] = None
        self.__entry['command'] = None

    def __set_callbacks(self):
        def slider_callback() -> None:
            self.__slider_edit_callback(self, self.__slider['value'])

        def entry_callback(s) -> None:
            try:
                f = float(s)
            except:
                return
            self.__entry_edit_callback(self, f)

        self.__slider['command'] = slider_callback
        self.__entry['command'] = entry_callback

    @property
    def frame(self) -> DirectFrame:
        return self.__frame

    def update_slider(self, value: float):
        if not self.__enabled:
            return
        self.__unset_callbacks()
        self.__slider['value'] = value
        self.__set_callbacks()

    def update_entry(self, value: float):
        if not self.__enabled:
            return
        self.__unset_callbacks()
        self.__entry.enterText(f'{value:.3f}')
        self.__set_callbacks()

    def update(self, value: float):
        self.update_slider(value)
        self.update_entry(value)

    @property
    def value(self) -> float:
        return self.__slider['value']

    @property
    def enabled(self) -> str:
        return 'enabled'

    @enabled.setter
    def enabled(self, enabled: bool) -> None:
        if self.__enabled == enabled:
            return
        self.__enabled = enabled
        if enabled:
            self.__disable_frame_overlay.hide()
        else:
            self.__disable_frame_overlay.show()

    @enabled.getter
    def enabled(self) -> bool:
        return self.__enabled


class AdvancedColourPicker():
    __base: ShowBase
    __colour: Colour
    __callback: Callable[[Colour], None]

    __frame: DirectFrame
    __colour_picker: ColourPicker
    __cv_picked: ColourView
    __cv_hovered: ColourView
    __r: ColourChannel
    __g: ColourChannel
    __b: ColourChannel
    __a: ColourChannel
    __dirty_rem_no_hide: int
    __enabled: bool

    def __init__(self, base: ShowBase, parent: DirectFrame, callback: Callable[[Colour], None],
                 mouse1_press_callbacks: List[Callable[[], None]],
                 colour: Colour = Colour(0.25, 0.5, 0.75, 1.0)):
        self.__base = base
        self.__colour = colour
        self.__callback = callback
        self.__enabled = True

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
        self.__r = ColourChannel(f, "R", c[0], sc, ec, mouse1_press_callbacks)
        self.__g = ColourChannel(f, "G", c[1], sc, ec, mouse1_press_callbacks)
        self.__b = ColourChannel(f, "B", c[2], sc, ec, mouse1_press_callbacks)
        self.__a = ColourChannel(f, "A", c[3], sc, ec, mouse1_press_callbacks)

        x = 0.14
        y_base = -0.8
        y_inc = -0.25
        self.__r.frame.set_pos((x, 0.0, y_base))
        self.__g.frame.set_pos((x, 0.0, y_base + y_inc))
        self.__b.frame.set_pos((x, 0.0, y_base + y_inc * 2))
        self.__a.frame.set_pos((x, 0.0, y_base + y_inc * 3))

        self.__dirty_rem_no_hide = 0

    def __colour_picked_callback(self, colour: Colour):
        n = 3
        r, g, b, _ = colour
        self.__r.update(r)
        self.__g.update(g)
        self.__b.update(b)
        self.__cv_picked.colour = Colour(r, g, b, self.__cv_picked.colour.a)
        self.__dirty_rem_no_hide = 3  # something fishy is going on
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

    @property
    def enabled(self) -> str:
        return 'enabled'

    @enabled.setter
    def enabled(self, enabled: bool) -> None:
        if self.__enabled == enabled:
            return
        self.__enabled = enabled
        if not enabled:
            self.colour = WINDOW_BG_COLOUR
        self.__colour_picker.enabled = enabled
        self.__r.enabled = enabled
        self.__g.enabled = enabled
        self.__b.enabled = enabled
        self.__a.enabled = enabled

    @enabled.getter
    def enabled(self) -> bool:
        return self.__enabled


class ViewElement():
    __name: str
    __visibility_callback: Callable[[str, bool], None]
    __colour_callback: Callable[[str, Colour], None]
    __cv_clicked_callback: Callable[['ViewElement'], None]
    __visible: bool

    __frame: DirectFrame
    __label: DirectLabel
    __cv: ColourView
    __cv_btn: DirectButton
    __visibility_btn: DirectButton
    __visibility_bar: DirectFrame

    def __init__(self, name: str, visibility_callback: Callable[[str, bool], None],
                 colour_callback: Callable[[str, Colour], None], cv_clicked_callback: Callable[['ViewElement'], None],
                 parent: DirectFrame, visible: bool = True, colour: Colour = Colour(0.2, 0.3, 0.4, 0.5),
                 mouse1_press_callbacks: List[Callable[[], None]] = []):
        self.__name = name
        self.__visible = visible
        self.__visibility_callback = None
        self.__colour_callback = None
        self.__cv_clicked_callback = None

        self.__frame = DirectFrame(parent=parent, frameColor=WINDOW_BG_COLOUR)

        self.__cv = ColourView(self.__frame, colour)
        self.__cv.frame.set_scale((0.15, 1.0, 0.15))
        self.__cv.frame.set_pos((-0.65, 1.0, 0.0))

        self.__cv_btn = DirectButton(parent=self.__cv.frame,
                                     frameColor=TRANSPARENT,
                                     borderWidth=(0, 0),
                                     frameSize=self.__cv.view["frameSize"],
                                     scale=self.__cv.view["scale"],
                                     pos=self.__cv.view["pos"],
                                     command=self.__cv_clicked)

        self.__label = DirectLabel(parent=self.__frame,
                                   text=self.__name,
                                   text_fg=WHITE,
                                   text_bg=WINDOW_BG_COLOUR,
                                   frameColor=WINDOW_BG_COLOUR,
                                   text_align=TextNode.ALeft,
                                   borderWidth=(.0, .0),
                                   pos=(-0.3, 0.0, -0.03),
                                   scale=(0.1, 1.0, 0.1))

        visibility_filename = os.path.join(GUI_DATA_PATH, "visible.png")
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

        self.visible = self.__visible  # trigger UI update

        self.__visibility_callback = visibility_callback
        self.__colour_callback = colour_callback
        self.__cv_clicked_callback = cv_clicked_callback

    def __toggle_visible(self):
        self.visible = not self.visible

    def __cv_clicked(self):
        if self.__cv_clicked_callback:
            self.__cv_clicked_callback(self)

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

    def destroy(self) -> None:
        self.__visibility_bar.destroy()
        self.__visibility_btn.destroy()
        self.__cv_btn.destroy()
        self.__cv.destroy()
        self.__label.destroy()
        self.__frame.destroy()


class ViewEditor():
    __services: Services
    __base: ShowBase
    __window: Window
    __colour_picker: AdvancedColourPicker
    __elems: List[ViewElement]

    def __init__(self, services: Services, mouse1_press_callbacks: List[Callable[[], None]]):
        self.__services = services
        self.__services.ev_manager.register_listener(self)
        self.__base = self.__services.graphics.window

        self.__window = Window(self.__base, "view_editor", mouse1_press_callbacks,
                               borderWidth=(0.0, 0.0),
                               frameColor=WINDOW_BG_COLOUR,
                               pos=(1.1, 0.5, 0.5),
                               frameSize=(-1.1, 1.1, -5.79, 1.56)
                               )

        self.__colour_picker = AdvancedColourPicker(self.__base, self.__window.frame, self.__colour_picker_callback, mouse1_press_callbacks)

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

        # ViewElements listing
        self.__elems_frame = DirectScrolledFrame(parent=self.__window.frame,
                                                 frameColor=WINDOW_BG_COLOUR,
                                                 frameSize=(-1, 1, -1.125, 1.1),
                                                 pos=(0, 0, -2.9),
                                                 autoHideScrollBars=True)

        # selected colour view frame
        self.__selected_cv_outline = DirectFrame(parent=self.__elems_frame.getCanvas(),
                                                 relief=DGG.SUNKEN,
                                                 frameColor=WHITE,
                                                 borderWidth=(0.15, 0.15),
                                                 frameSize=(-0.62, 0.62, -0.54, 0.54),
                                                 scale=(0.18, 1.0, 0.18))
        # selected view frame
        self.__selected_view_outline = DirectFrame(parent=self.__window.frame,
                                                   relief=DGG.SUNKEN,
                                                   frameColor=WHITE,
                                                   borderWidth=(0.15, 0.15),
                                                   frameSize=(-0.62, 0.62, -0.54, 0.54),
                                                   scale=(0.3, 2.0, 0.35))

        self.heading = DirectLabel(parent=self.__window.frame,
                                   text="View Editor",
                                   text_fg=WHITE,
                                   text_bg=WINDOW_BG_COLOUR,
                                   frameColor=WINDOW_BG_COLOUR,
                                   borderWidth=(.0, .0),
                                   pos=(-0.42, 0.0, 1.27),
                                   scale=(0.2, 3, 0.2))

        self.__save_outline = DirectFrame(parent=self.__window.frame,
                                          frameColor=WHITE,
                                          pos=(-0.57, 0, -5.45),
                                          borderWidth=(0.25, 0.15),
                                          frameSize=(-0.62, 0.62, -0.54, 0.54),
                                          scale=(0.50, 2.1, 0.25))

        self.__restore_outline = DirectFrame(parent=self.__window.frame,
                                             frameColor=WHITE,
                                             pos=(0.50, 0, -5.45),
                                             borderWidth=(0.25, 0.15),
                                             frameSize=(-0.62, 0.62, -0.54, 0.54),
                                             scale=(0.65, 2.1, 0.25))

        # save and restore
        self.btn_s = DirectButton(
            text="Save",
            text_fg=(0.3, 0.3, 0.3, 1.0),
            pressEffect=1,
            command=self.__save,
            pos=(-0.575, 0, -5.5),
            parent=self.__window.frame,
            scale=(0.20, 2.1, 0.15),
            frameColor=TRANSPARENT)

        self.btn_r = DirectButton(
            text="Restore",
            text_fg=(0.3, 0.3, 0.3, 1.0),
            pressEffect=1,
            command=self.__reset,
            pos=(0.50, 0, -5.5),
            parent=self.__window.frame,
            scale=(0.20, 2.1, 0.15),
            frameColor=TRANSPARENT)

        # zoom window in / out
        self.btn_zoom_out = DirectButton(
            text="-",
            text_fg=WHITE,
            pressEffect=1,
            command=self.__window.zoom_out,
            pos=(0.5, 0., 1.25),
            parent=self.__window.frame,
            scale=(0.38, 4.25, 0.45),
            frameColor=TRANSPARENT)

        self.btn_zoom_in = DirectButton(
            text="+",
            text_fg=WHITE,
            pressEffect=1,
            command=self.__window.zoom_in,
            pos=(0.71, 0., 1.28),
            parent=self.__window.frame,
            scale=(0.35, 4.19, 0.38),
            frameColor=TRANSPARENT)

        # Quit button
        self.btn = DirectButton(text='x',
                                text_fg=WHITE,
                                command=self.__window.toggle_visible,
                                pos=(0.91, 0.4, 1.3),
                                parent=self.__window.frame,
                                scale=(0.3, 2.9, 0.2),
                                pressEffect=1,
                                frameColor=TRANSPARENT)

        # Creating view selectors
        self.__view_selectors = []
        for i in range(0, PersistentStateViews.MAX_VIEWS):
            num = os.path.join(GUI_DATA_PATH, str(i + 1) + ".png")

            self.__view_selectors.append(DirectButton(image=num,
                                                      pos=(-0.7 + (i % 3) * 0.7, 0.4, -4.4 - 0.5 * (i // 3)),
                                                      parent=self.__window.frame,
                                                      scale=0.16,
                                                      frameColor=TRANSPARENT,
                                                      command=lambda v: setattr(self, "view_idx", v),
                                                      extraArgs=[i]))

        self.__elems = []
        self.__reset()

    def __reset(self) -> None:
        self.__services.state.views.restore_effective_view()
        ps_colours = self.__services.state.views.effective_view.colours

        for e in self.__elems:
            e.destroy()
        self.__elems.clear()

        for name, dc in ps_colours.items():
            self.__elems.append(ViewElement(name, self.__update_visibility, self.__update_colour, self.__select_cv,
                                            self.__elems_frame.getCanvas(), dc.visible, dc.colour))

        for i in range(len(self.__elems)):
            self.__elems[i].frame.set_pos((0.15, 0.0, -0.2 * i))
        self.__elems_frame["canvasSize"] = (-0.95, 0.95, -0.2 * len(self.__elems) + 0.1, 0.1)

        self.__cur_view_idx = self.view_idx
        self.__selected_view_outline.set_pos((-0.7 + (self.view_idx % 3) * 0.7, 0.4, -4.4 - 0.5 * (self.view_idx // 3)))

        self.__select_cv(self.__elems[0] if self.__elems else None)

    def __save(self) -> None:
        self.__services.state.views.apply_effective_view()

    def __update_visibility(self, name: str, visible: bool) -> None:
        self.__services.state.views.effective_view.colours[name].visible = visible

    def __update_colour(self, name: str, colour: Colour) -> None:
        self.__services.state.views.effective_view.colours[name].colour = colour

    def __colour_picker_callback(self, colour: Colour) -> None:
        self.__window.focus()
        if self.__selected_cv_elem is not None:
            self.__selected_cv_elem.colour = colour  # calls self.__update_colour()

    def __select_cv(self, e: ViewElement):
        self.__window.focus()
        self.__selected_cv_elem = e
        if e is None:
            self.__selected_cv_outline.hide()
            self.__colour_picker.enabled = False
        else:
            self.__colour_picker.enabled = True
            self.__selected_cv_outline.show()
            self.__selected_cv_outline.set_pos((-0.495, 1.0, -0.2 * self.__elems.index(e)))
            self.__colour_picker.colour = e.colour

    def notify(self, event: Event) -> None:
        if isinstance(event, NewColourEvent) or self.__services.state.views.view_idx != self.__cur_view_idx:
            self.__reset()
        elif isinstance(event, ToggleViewEvent):
            self.__window.toggle_visible()

    @property
    def view_idx(self) -> str:
        return 'view_idx'

    @view_idx.setter
    def view_idx(self, idx: int) -> None:
        self.__services.state.views.view_idx = idx
        self.__reset()

    @view_idx.getter
    def view_idx(self) -> int:
        return self.__services.state.views.view_idx
