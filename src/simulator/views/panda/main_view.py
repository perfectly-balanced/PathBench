from direct.showbase.ShowBase import ShowBase
from panda3d.core import PointLight, AmbientLight, LVector4
from cube_mesh_generator import CubeMeshGenerator
from typing import List

assert(__name__ != "__main__")

class MainView(ShowBase):
    def __init__(self, map3d : List[List[List[bool]]]) -> None:
        super().__init__(self)

        self.set_background_color(0,0,0,1)
        self.cam.setPos(0, -40, 0)

        self.__map = map3d
        self.__mesh = CubeMeshGenerator('3D Voxel Map')
        
        for j in self.__map:
            for i in self.__map[j]:
                for k in self.__map[j][i]:
                    if not self.__map[j][i][k]:
                        continue
                        
                    if j-1 not in self.__map or not self.__map[j-1][i][k]:
                        self.__mesh.make_left_face(j, i, k)
                    if j+1 not in self.__map or not self.__map[j+1][i][k]:
                        self.__mesh.make_right_face(j, i, k)
                    if i-1 not in self.__map[j] or not self.__map[j][i-1][k]:
                        self.__mesh.make_back_face(j, i, k)
                    if i+1 not in self.__map[j] or not self.__map[j][i+1][k]:
                        
                        self.__mesh.make_front_face(j, i, k)
                    if k-1 not in self.__map[j][i] or not self.__map[j][i][k-1]:
                        self.__mesh.make_bottom_face(j, i, k) 
                    if k+1 not in self.__map[j][i] or not self.__map[j][i][k+1]:
                        self.__mesh.make_top_face(j, i, k)
                        
        terrain = self.render.attachNewNode(self.__mesh.geom_node)

        # LIGHTING #
        
        self.light_model = self.loader.loadModel('models/misc/sphere')
        self.light_model.setScale(0.2, 0.2, 0.2)
        self.light_model.setPos(25, -25, 25)
        self.light_model.reparentTo(self.render)

        plight = PointLight("plight")
        plight.setShadowCaster(True, 2048, 2048)
        plnp = self.light_model.attachNewNode(plight)
        plight.setAttenuation((1, 0, 0)) # constant, linear, and quadratic.
        terrain.setLight(plnp)
        
        alight = AmbientLight("alight")
        alight.setColor((0.04, 0.04, 0.04, 1))
        alnp = self.render.attachNewNode(alight)
        terrain.setLight(alnp)

        terrain.setShaderAuto()