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

@unique
class Lighting(IntEnum):
    ARTIFICAL = 0
    BASIC = 1
    CUSTOM = 2


START_CUBE_COLOUR: Final = GREEN
GOAL_CUBE_COLOUR: Final = RED

class MainView(ShowBase):
    __start_pos: IntPoint3
    __goal_pos: IntPoint3

    def __init__(self, map_data: List[List[List[bool]]], start_pos: IntPoint3 = None, goal_pos: IntPoint3 = None, lighting: Lighting = Lighting.ARTIFICAL) -> None:
        super().__init__(self)

        # disables the default camera behaviour
        self.disable_mouse()


        self.set_background_color(0, 0, 0.2, 1)

        # Creating the world origin as a dummy node
        self.worldOrigin = self.render.attachNewNode("Origin")

        # MAP #
        self.__traversables_map_data = map_data

        self.__obstacles_map_data = {}
        for i in range(len(self.__traversables_map_data)):
            self.__obstacles_map_data[i] = {}
            for j in range(len(self.__traversables_map_data[i])):
                self.__obstacles_map_data[i][j] = {}
                for k in range(len(self.__traversables_map_data[i][j])):
                    self.__obstacles_map_data[i][j][k] = False if self.__traversables_map_data[i][j][k] else True

        self.__generate_map(lighting == Lighting.ARTIFICAL)
        self.__map.reparentTo(self.worldOrigin)
        self.__map.setPos(self.worldOrigin.getX() - len(self.__traversables_map_data) / 2, self.worldOrigin.getY() - len(self.__traversables_map_data) / 2, self.worldOrigin.getZ() - len(self.__traversables_map_data) / 2)

        # CAMERA #
        self.__camera = Camera(self, self.cam, self.worldOrigin)

        # LIGHTING #
        self.__ambient = 0.5
        if lighting == Lighting.BASIC:
            self.__basic_lighting()
        elif lighting == Lighting.CUSTOM:
            self.__custom_lighting()

        # initialise goal and origin #
        self.__start_pos = None
        self.__goal_pos = None
        self.start_pos = start_pos
        self.goal_pos = goal_pos

        # randomly initialise undefined positions
        if self.goal_pos == None or self.start_pos == None:
            map_sz = 0
            for i in range(len(self.__traversables_map_data)):
                for j in range(len(self.__traversables_map_data[i])):
                    map_sz += len(self.__traversables_map_data[i][j])

            def randpos() -> IntPoint3:
                def to_coord(n: int) -> IntPoint3:
                    for i in range(len(self.__traversables_map_data)):
                        for j in range(len(self.__traversables_map_data[i])):
                            for k in range(len(self.__traversables_map_data[i][j])):
                                if n == 0:
                                    return (i, j, k)
                                else:
                                    n -= 1

                def cube_exists(n: int) -> bool:
                    i, j, k = to_coord(n)
                    return self.__traversables_map_data[i][j][k]

                def gen() -> int:
                    return random.randint(0, map_sz-1)

                n = gen()

                while not cube_exists(n):
                    n = gen()
                return to_coord(n)

            if self.start_pos == None:
                p = randpos()
                while p == self.goal_pos:
                    p = randpos()
                self.start_pos = p

            if self.goal_pos == None:
                p = randpos()
                while p == self.start_pos:
                    p = randpos()
                self.goal_pos = p
        
        # collision traverser & queue
        self.__ctrav = CollisionTraverser('ctrav')
        self.__cqueue = CollisionHandlerQueue()

        # map collision boxes
        self.__map_cn = CollisionNode('3D Voxel Map Collision Node')
        self.__map_cn.set_collide_mask(BitMask32.bit(1))
        self.__map_cnp = self.__map.attach_new_node(self.__map_cn)
        self.__ctrav.add_collider(self.__map_cnp, self.__cqueue)
        
        for x in range(len(self.__traversables_map_data)):
            for y in range(len(self.__traversables_map_data[x])):
                for z in range(len(self.__traversables_map_data[x][y])):
                    if self.__traversables_map_mesh.cube_visible((x, y, z)):
                        self.__map_cn.add_solid(CollisionBox(Point3(x,y,z), Point3(x+1, y+1, z-1)))

        # mouse picker
        picker_node = CollisionNode('mouseRay')
        picker_np = self.cam.attach_new_node(picker_node)
        picker_node.set_from_collide_mask(BitMask32.bit(1))
        
        self.picker_ray = CollisionRay()
        picker_node.add_solid(self.picker_ray)
        self.__ctrav.add_collider(picker_np, self.__cqueue)

        # debug -> shows collision ray / impact point
        # self.__ctrav.show_collisions(self.__map)

        def on_click():
            # check if we have access to the mouse
            if self.mouseWatcherNode.hasMouse():
                # get the mouse position
                mpos = self.mouseWatcherNode.get_mouse()

                # set the position of the ray based on the mouse position
                self.picker_ray.set_from_lens(self.camNode,mpos.getX(),mpos.getY())
                self.__ctrav.traverse(self.__map)
                # if we have hit something sort the hits so that the closest is first and highlight the node
                if self.__cqueue.get_num_entries() > 0:
                    self.__cqueue.sort_entries()
                    po = self.__cqueue.get_entry(0).get_into_node_path()

                    x, y, z = self.__cqueue.get_entry(0).getSurfacePoint(self.__map)
                    pos = [max(math.floor(x), 0), max(math.floor(y), 0), max(math.ceil(z), 0)]
                    if pos[0] == len(self.__traversables_map_data):
                        pos[0] -= 1
                    if pos[1] == len(self.__traversables_map_data[pos[0]]):
                        pos[1] -= 1
                    if pos[2] == len(self.__traversables_map_data[pos[0]][pos[1]]):
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
                self.goal_pos = p
        
        def right_click():
            p = on_click()
            if p != None:
                self.start_pos = p

        self.accept('mouse1', left_click)
        self.accept('mouse3', right_click)

        # GUI #
        self.__vs = ViewEditor(self, self.__traversables_map, self.__traversables_map_wf, self.__obstacles_map, self.__obstacles_map_wf, self.__traversables_map_mesh, self.__traversables_map_wf_mesh, self.__obstacles_map_mesh, self.__obstacles_map_wf_mesh)

    @property
    def start_pos(self) -> str:
        return 'start_pos'

    @start_pos.getter
    def start_pos(self) -> IntPoint3:
        return self.__start_pos

    @start_pos.setter
    def start_pos(self, value: IntPoint3) -> None:
        if self.__start_pos != None:
            self.__traversables_map_mesh.reset_cube_colour(self.__start_pos)
        self.__start_pos = value
        if self.__start_pos != None:
            self.__traversables_map_mesh.set_cube_colour(self.__start_pos, START_CUBE_COLOUR)

    @property
    def goal_pos(self) -> str:
        return 'goal_pos'

    @goal_pos.getter
    def goal_pos(self) -> IntPoint3:
        return self.__goal_pos

    @goal_pos.setter
    def goal_pos(self, value: IntPoint3) -> None:
        if self.__goal_pos != None:
            self.__traversables_map_mesh.reset_cube_colour(self.__goal_pos)
        self.__goal_pos = value
        if self.__goal_pos != None:
            self.__traversables_map_mesh.set_cube_colour(self.__goal_pos, GOAL_CUBE_COLOUR)

    def __generate_map(self, artificial_lighting: bool = False) -> None:
        self.__map = self.render.attach_new_node("3D Voxel Map")

        self.__traversables_map_mesh = CubeMesh(self.__traversables_map_data, '3D Voxel Traversables Map', artificial_lighting, default_colour = WHITE, hidden_faces = True)
        self.__traversables_map = self.__map.attach_new_node(self.__traversables_map_mesh.geom_node)
        self.__traversables_map.set_transparency(TransparencyAttrib.M_alpha)

        # Wireframe overlay
        self.__traversables_map_wf_mesh = CubeMesh(self.__traversables_map_data, '3D Voxel Traversables Map Wireframe', artificial_lighting, default_colour = BLACK, hidden_faces = False)
        self.__traversables_map_wf = self.__map.attach_new_node(self.__traversables_map_wf_mesh.geom_node)
        self.__traversables_map_wf.setRenderModeWireframe()

        self.__obstacles_map_mesh = CubeMesh(self.__obstacles_map_data, '3D Voxel Obstacles Map', artificial_lighting, default_colour = WHITE, hidden_faces = True)
        self.__obstacles_map = self.__map.attach_new_node(self.__obstacles_map_mesh.geom_node)
        self.__obstacles_map.set_transparency(TransparencyAttrib.M_alpha)

        # Wireframe overlay
        self.__obstacles_map_wf_mesh = CubeMesh(self.__obstacles_map_data, '3D Voxel Obstacles Map Wireframe', artificial_lighting, default_colour = BLACK, hidden_faces = False)
        self.__obstacles_map_wf = self.__map.attach_new_node(self.__obstacles_map_wf_mesh.geom_node)
        self.__obstacles_map_wf.setRenderModeWireframe()


    def __basic_lighting(self) -> None:
        # POINT LIGHT #
        plight = PointLight("plight")
        plnp = self.cam.attach_new_node(plight)
        plight.set_shadow_caster(True, 2048, 2048)
        # plight.set_attenuation((1.0, 0, 0)) # constant, linear, and quadratic.
        self.__map.set_light(plnp)

        bmin, bmax = self.__map.get_tight_bounds(plnp)
        lens = plight.get_lens()
        lens.set_film_offset((bmin.xz + bmax.xz) * 0.5)
        lens.set_film_size(bmax.xz - bmin.xz)
        lens.set_near_far(bmin.y, bmax.y)

        # AMBIENT #
        alight = AmbientLight("alight")
        alnp = self.cam.attach_new_node(alight)
        alight.set_color((self.__ambient, self.__ambient, self.__ambient, 1.0))
        self.__map.set_light(alnp)

        # ENABLE DEFAULT SHADOW SHADERS #
        self.__map.set_shader_auto()

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
        self.__light_cam.look_at(self.__map)
        self.__light_cam.node().get_lens().set_near_far(10, 1000)
        self.__light_cam.node().hideFrustum()

