from direct.showbase.ShowBase import ShowBase
from cube_mesh_generator import CubeMeshGenerator
import random

class Viewer(ShowBase):
    def __init__(self):
        super().__init__(self)
        self.__map = {}
 
        for x in range(0, 25):
            self.__map[x] = {}
            for y in range(0, 25):
                self.__map[x][y] = {}
                for z in range(0, 25):
                    self.__map[x][y][z] = bool(random.getrandbits(1))
        
        self.__mesh = CubeMeshGenerator('3D Voxel Map')
        
        for j in self.__map:
            for i in self.__map[j]:
                for k in self.__map[j][i]:
                    if not self.__map[j][i][k]:
                        continue
                        
                    if j-1 not in self.__map or self.__map[j-1][i][k] == 0:
                        self.__mesh.make_left_face(j, i, k)
                    if j+1 not in self.__map or self.__map[j+1][i][k] == 0:
                        self.__mesh.make_right_face(j, i, k)
                    if i-1 not in self.__map[j] or self.__map[j][i-1][k] == 0:
                        self.__mesh.make_back_face(j, i, k)
                    if i+1 not in self.__map[j] or self.__map[j][i+1][k] == 0:
                        
                        self.__mesh.make_front_face(j, i, k)
                    if k-1 not in self.__map[j][i] or self.__map[j][i][k-1] == 0:
                        self.__mesh.make_bottom_face(j, i, k) 
                    if k+1 not in self.__map[j][i] or self.__map[j][i][k+1] == 0:
                        self.__mesh.make_top_face(j, i, k)
                        
        cube = render.attachNewNode(self.__mesh.geom_node)

view = Viewer()
view.run()