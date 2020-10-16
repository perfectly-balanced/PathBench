from direct.showbase.PythonUtil import clampScalar
from direct.showbase.ShowBase import ShowBase
from panda3d.core import PointLight, AmbientLight, LPoint3, WindowProperties, FrameBufferProperties, GraphicsPipe, Texture, GraphicsOutput, NodePath, PandaNode
from panda3d.core import *

from enum import IntEnum, unique
from typing import List, Tuple, Final
import random
import math

from . import camera
from .camera import Camera
from .cube_mesh import CubeMesh
from .common import IntPoint3, Colour, WHITE, BLACK, RED, GREEN, BLUE
from .gui.view_editor import ViewEditor
from .voxel_map import VoxelMap

@unique
class Lighting(IntEnum):
    ARTIFICAL = 0
    BASIC = 1
    CUSTOM = 2


START_CUBE_COLOUR: Final = GREEN
GOAL_CUBE_COLOUR: Final = RED

class MainView(ShowBase):
    __map: VoxelMap

    def __init__(self, map_data: List[List[List[bool]]], start_pos: IntPoint3 = None, goal_pos: IntPoint3 = None, lighting: Lighting = Lighting.ARTIFICAL) -> None:
        super().__init__(self)

        # disables the default camera behaviour
        self.disable_mouse()


        self.set_background_color(0, 0, 0.2, 1)

        # Creating the world origin as a dummy node
        self.world_origin = self.render.attach_new_node("origin")

        # MAP #
        self.__map = VoxelMap(map_data, self.world_origin, start_pos=start_pos, goal_pos=goal_pos, artificial_lighting=(lighting == Lighting.ARTIFICAL))
        self.__map.root.set_pos(self.world_origin.getX() - len(self.map.traversables_data) / 2, self.world_origin.getY() - len(self.map.traversables_data) / 2, self.world_origin.getZ() - len(self.map.traversables_data) / 2)

        # CAMERA #
        self.__camera = Camera(self, self.cam, self.world_origin)

        # LIGHTING #
        self.__ambient = 0.5
        if lighting == Lighting.BASIC:
            self.__basic_lighting()
        elif lighting == Lighting.CUSTOM:
            self.__custom_lighting()
        
        # collision traverser & queue
        self.__ctrav = CollisionTraverser('ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # map collision boxes
        self.__map_cn = CollisionNode('3D Voxel Map Collision Node')
        self.__map_cn.set_collide_mask(BitMask32.bit(1))
        self.__map_cnp = self.map.traversables.attach_new_node(self.__map_cn)
        self.__ctrav.add_collider(self.__map_cnp, self.__cqueue)
        
        for x in range(len(self.map.traversables_data)):
            for y in range(len(self.map.traversables_data[x])):
                for z in range(len(self.map.traversables_data[x][y])):
                    if self.map.traversables_mesh.cube_visible((x, y, z)):
                        self.__map_cn.add_solid(CollisionBox(Point3(x,y,z), Point3(x+1, y+1, z-1)))

        # mouse picker
        picker_node = CollisionNode('mouseRay')
        picker_np = self.cam.attach_new_node(picker_node)
        picker_node.set_from_collide_mask(BitMask32.bit(1))
        
        self.picker_ray = CollisionRay()
        picker_node.add_solid(self.picker_ray)
        self.__ctrav.add_collider(picker_np, self.__cqueue)

        # debug -> shows collision ray / impact point
        # self.__ctrav.show_collisions(self.map.root)

        def on_click():
            # check if we have access to the mouse
            if self.mouseWatcherNode.hasMouse():
                # get the mouse position
                mpos = self.mouseWatcherNode.get_mouse()

                # set the position of the ray based on the mouse position
                self.picker_ray.set_from_lens(self.camNode,mpos.getX(),mpos.getY())
                self.__ctrav.traverse(self.map.traversables)
                # if we have hit something sort the hits so that the closest is first and highlight the node
                if self.__cqueue.get_num_entries() > 0:
                    self.__cqueue.sort_entries()
                    po = self.__cqueue.get_entry(0).get_into_node_path()

                    x, y, z = self.__cqueue.get_entry(0).getSurfacePoint(self.map.traversables)
                    pos = [max(math.floor(x), 0), max(math.floor(y), 0), max(math.ceil(z), 0)]
                    if pos[0] == len(self.map.traversables_data):
                        pos[0] -= 1
                    if pos[1] == len(self.map.traversables_data[pos[0]]):
                        pos[1] -= 1
                    if pos[2] == len(self.map.traversables_data[pos[0]][pos[1]]):
                        pos[2] -= 1
                    pos = tuple(pos)
                    
                    # debug
                    import time
                    print(str(time.time()) + ' click on ' + po.get_name() + ' pos ' + str(pos))
                    
                    return pos
            return None
        
        def left_click():
            p = on_click()
            if p != None:
                self.map.goal_pos = p
        
        def right_click():
            p = on_click()
            if p != None:
                self.map.start_pos = p

        self.accept('mouse1', left_click)
        self.accept('mouse3', right_click)

        # GUI #
        self.__vs = ViewEditor(self, self.map)

    @property
    def map(self) -> str:
        return 'map'

    @map.getter
    def map(self) -> VoxelMap:
        return self.__map

    def __basic_lighting(self) -> None:
        # POINT LIGHT #
        plight = PointLight("plight")
        plnp = self.cam.attach_new_node(plight)
        plight.set_shadow_caster(True, 2048, 2048)
        # plight.set_attenuation((1.0, 0, 0)) # constant, linear, and quadratic.
        self.map.root.set_light(plnp)

        bmin, bmax = self.map.root.get_tight_bounds(plnp)
        lens = plight.get_lens()
        lens.set_film_offset((bmin.xz + bmax.xz) * 0.5)
        lens.set_film_size(bmax.xz - bmin.xz)
        lens.set_near_far(bmin.y, bmax.y)

        # AMBIENT #
        alight = AmbientLight("alight")
        alnp = self.cam.attach_new_node(alight)
        alight.set_color((self.__ambient, self.__ambient, self.__ambient, 1.0))
        self.map.root.set_light(alnp)

        # ENABLE DEFAULT SHADOW SHADERS #
        self.map.root.set_shader_auto()

    def __custom_lighting(self) -> None:
        # preliminary capabilities check
        if not self.win.getGsg().get_supports_basic_shaders():
            raise Exception("Video driver reports that shaders are not supported.")
        if not self.win.getGsg().get_supports_depth_texture():
            raise Exception("Video driver reports that depth textures are not supported.")

        # creating the offscreen light buffer
        winprops = WindowProperties(size=self.get_size())
        props = FrameBufferProperties()
        props.set_rgb_color(1)
        props.set_alpha_bits(1)
        props.set_depth_bits(1)
        self.__light_buffer = self.graphics_engine.make_output(
            self.pipe, "light_buffer", -2,
            props, winprops,
            GraphicsPipe.BFRefuseWindow,
            self.win.getGsg(), self.win)

        if not self.__light_buffer:
            raise Exception("Video driver cannot create an offscreen buffer.")

        self.__light_depth_map = Texture()
        self.__light_buffer.add_render_texture(self.__light_depth_map, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPDepthStencil)
        if self.win.getGsg().get_supports_shadow_filter():
            self.__light_depth_map.setMinfilter(Texture.FTShadow)
            self.__light_depth_map.setMagfilter(Texture.FTShadow)

        # adding a color texture is totally unnecessary, but it helps with debugging.
        self.__light_colour_map = Texture()
        self.__light_buffer.add_render_texture(self.__light_colour_map, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPColor)

        # light is controlled via this camera
        self.__light_cam = self.makeCamera(self.__light_buffer, camName="light_cam", useCamera=self.cam)

        # setting up shader
        self.render.set_shader_input('light', self.__light_cam)
        self.render.set_shader_input('light_depth_map', self.__light_depth_map)
        self.render.set_shader_input('ambient', self.__ambient)
        self.render.set_shader_input('tex_disable', (1, 1, 1, 1))
        self.render.set_shader_input('scale', (1, 1, 1, 1))
        self.render.set_shader_input('push', 1.0)

        # Put a shader on the Light camera.
        lci = NodePath(PandaNode("Light Camera Initializer"))
        lci.set_shader(loader.load_shader('shader/caster.sha'))
        self.__light_cam.node().set_initial_state(lci.get_state())

        # Put a shader on the Main camera.
        # Some video cards have special hardware for shadow maps.
        # If the card has that, use it.  If not, use a different
        # shader that does not require hardware support.

        mci = NodePath(PandaNode("Main Camera Initializer"))
        if self.win.getGsg().get_supports_shadow_filter():
            mci.set_shader(loader.load_shader('shader/shadow.sha'))
        else:
            mci.set_shader(loader.load_shader('shader/shadow-nosupport.sha'))
        self.cam.node().set_initial_state(mci.get_state())

        # setup light camera
        self.__light_cam.look_at(self.map.root)
        self.__light_cam.node().get_lens().set_near_far(10, 1000)
        self.__light_cam.node().hideFrustum()

