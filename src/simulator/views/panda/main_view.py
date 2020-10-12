from direct.showbase.ShowBase import ShowBase
from panda3d.core import PointLight, AmbientLight, LVector4, LPoint3, WindowProperties, FrameBufferProperties, GraphicsPipe, Texture, GraphicsOutput, NodePath, PandaNode
from cube_mesh_generator import CubeMeshGenerator
from typing import List

assert(__name__ != "__main__")

from enum import IntEnum, unique

@unique
class Lighting(IntEnum):
    ARTIFICAL = 0
    BASIC = 1
    CUSTOM = 2

class MainView(ShowBase):
    def __init__(self, map3d: List[List[List[bool]]], lighting: Lighting = Lighting.ARTIFICAL) -> None:
        super().__init__(self)
        
        self.set_background_color(0, 0, 0.2, 1)

        # MAP #
        self.__map_data = map3d
        self.__generate_map(lighting == Lighting.ARTIFICAL)

        # VIEW #
        self.cam.set_pos(0, -25, 25)
        self.cam.look_at(self.__map)

        # LIGHTING #
        self.__ambient = 0.5
        if lighting == Lighting.BASIC:
            self.__basic_lighting()
        elif lighting == Lighting.CUSTOM:
            self.__custom_lighting()
    
    def __generate_map(self, artificial_lighting: bool = False) -> None:
        self.__mesh = CubeMeshGenerator('3D Voxel Map', artificial_lighting)

        for j in self.__map_data:
            for i in self.__map_data[j]:
                for k in self.__map_data[j][i]:
                    if not self.__map_data[j][i][k]:
                        continue
                        
                    if j-1 not in self.__map_data or not self.__map_data[j-1][i][k]:
                        self.__mesh.make_left_face(j, i, k)
                    if j+1 not in self.__map_data or not self.__map_data[j+1][i][k]:
                        self.__mesh.make_right_face(j, i, k)
                    if i-1 not in self.__map_data[j] or not self.__map_data[j][i-1][k]:
                        self.__mesh.make_back_face(j, i, k)
                    if i+1 not in self.__map_data[j] or not self.__map_data[j][i+1][k]:
                        self.__mesh.make_front_face(j, i, k)
                    if k-1 not in self.__map_data[j][i] or not self.__map_data[j][i][k-1]:
                        self.__mesh.make_bottom_face(j, i, k) 
                    if k+1 not in self.__map_data[j][i] or not self.__map_data[j][i][k+1]:
                        self.__mesh.make_top_face(j, i, k)
                        
        self.__map = self.render.attach_new_node(self.__mesh.geom_node)
        self.__map_movement = self.__map.hprInterval(50, LPoint3(0, 360, 360))
        self.__map_movement.loop()

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