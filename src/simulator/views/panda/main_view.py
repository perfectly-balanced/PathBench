from direct.showbase.ShowBase import ShowBase
from panda3d.core import PointLight, AmbientLight, LVector4, LPoint3, WindowProperties, FrameBufferProperties, GraphicsPipe, Texture, GraphicsOutput, NodePath, PandaNode
from cube_mesh_generator import CubeMeshGenerator
from typing import List

assert(__name__ != "__main__")

class MainView(ShowBase):
    def __init__(self, map3d : List[List[List[bool]]]) -> None:
        super().__init__(self)

        self.disableMouse()
        self.setBackgroundColor(0, 0, 0.2, 1)

        # MAP #
        self.__map_data = map3d
        self.__generate_map()

        # LIGHTING #
        self.ambient = 0.2
        # self.basic_lighting()
        self.advanced_lighting()

        # VIEW #
        self.cam.reparentTo(self.render)
        self.cam.setPos(0, -25, 25)
        self.cam.lookAt(self.__map)
    
    def __generate_map(self):
        self.__mesh = CubeMeshGenerator('3D Voxel Map')

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
                        
        self.__map = self.render.attachNewNode(self.__mesh.geom_node)
        self.__map_movement = self.__map.hprInterval(50, LPoint3(0, 360, 360))
        self.__map_movement.loop()

    def basic_lighting(self):
        # POINT LIGHT #
        plight = PointLight("plight")
        plnp = self.cam.attachNewNode(plight)
        plight.setShadowCaster(True, 2048, 2048)
        plight.setAttenuation((1, 0, 0)) # constant, linear, and quadratic.
        plight.getLens().setFov(40)
        plight.getLens().setNearFar(10, 2000)
        self.__map.setLight(plnp)
        
        # AMBIENT #
        alight = AmbientLight("alight")
        alnp = self.render.attachNewNode(alight)
        alight.setColor((self.ambient, self.ambient, self.ambient, 1.0))
        self.__map.setLight(alnp)

        # ENABLE DEFAULT SHADOW SHADERS #
        self.__map.setShaderAuto()
    
    def advanced_lighting(self):
        # Preliminary capabilities check.
        if not self.win.getGsg().getSupportsBasicShaders():
            raise Exception("Video driver reports that shaders are not supported.")
        if not self.win.getGsg().getSupportsDepthTexture():
            raise Exception("Video driver reports that depth textures are not supported.")

        # creating the offscreen buffer.
        winprops = WindowProperties(size=(512, 512))
        props = FrameBufferProperties()
        props.setRgbColor(1)
        props.setAlphaBits(1)
        props.setDepthBits(1)
        self.__light_buffer = self.graphicsEngine.makeOutput(
            self.pipe, "offscreen buffer", -2,
            props, winprops,
            GraphicsPipe.BFRefuseWindow,
            self.win.getGsg(), self.win)
        self.__light_buffer.active = True

        if not self.__light_buffer:
            raise Exception("Video driver cannot create an offscreen buffer.")

        self.__light_depth_map = Texture()
        self.__light_buffer.addRenderTexture(self.__light_depth_map, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPDepthStencil)
        if self.win.getGsg().getSupportsShadowFilter():
            self.__light_depth_map.setMinfilter(Texture.FTShadow)
            self.__light_depth_map.setMagfilter(Texture.FTShadow)

        # Adding a color texture is totally unnecessary, but it helps with debugging.
        self.__light_colour_map = Texture()
        self.__light_buffer.addRenderTexture(self.__light_colour_map, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPColor)

        self.__light_cam = self.makeCamera(self.__light_buffer)
        self.__light_cam.node().setScene(self.render)
        self.__light_cam.node().getLens().setFov(40)

        # setting up shader
        self.render.setShaderInput('light', self.__light_cam)
        self.render.setShaderInput('Ldepthmap', self.__light_depth_map)
        self.render.setShaderInput('ambient', (self.ambient, 0, 0, 1.0))
        self.render.setShaderInput('texDisable', (1, 1, 1, 1))
        self.render.setShaderInput('scale', (1, 1, 1, 1))

        # Put a shader on the Light camera.
        lci = NodePath(PandaNode("Light Camera Initializer"))
        lci.setShader(loader.loadShader('shader/caster.sha'))
        self.__light_cam.node().setInitialState(lci.getState())

        # Put a shader on the Main camera.
        # Some video cards have special hardware for shadow maps.
        # If the card has that, use it.  If not, use a different
        # shader that does not require hardware support.

        mci = NodePath(PandaNode("Main Camera Initializer"))
        if self.win.getGsg().getSupportsShadowFilter():
            mci.setShader(loader.loadShader('shader/shadow.sha'))
        else:
            mci.setShader(loader.loadShader('shader/shadow-nosupport.sha'))
        self.cam.node().setInitialState(mci.getState())

        self.__light_cam.setPos(0, -25, 25)
        self.__light_cam.lookAt(self.__map)
        self.__light_cam.node().getLens().setNearFar(10, 1000)
        self.__light_cam.node().hideFrustum()

        self.render.setShaderInput('push', 0.5)