from panda3d.core import Geom, GeomNode, NodePath, AntialiasAttrib, GeomVertexFormat, GeomVertexData, GeomPoints

import numpy as np

class PointCloud():
    name: str
    node_path: NodePath
    geom_node: GeomNode

    def __init__(self, name='point_cloud', thickness: float = 3) -> None:
        self.name = name
        self.geom_node = GeomNode(self.name)
        self.node_path = NodePath(self.geom_node)
        self.node_path.set_render_mode_wireframe()
        self.node_path.set_render_mode_thickness(thickness)
        self.node_path.set_antialias(AntialiasAttrib.MPoint)

    @staticmethod
    def __make_points(vertices, colours=None, geom: Geom = None) -> Geom:
        """
        This function is largely inspired by panda3d_viewer's implementation.

        Copyright (c) 2020, Igor Kalevatykh

        Permission is hereby granted, free of charge, to any person obtaining a copy
        of this software and associated documentation files (the "Software"), to deal
        in the Software without restriction, including without limitation the rights
        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
        copies of the Software, and to permit persons to whom the Software is
        furnished to do so, subject to the following conditions:

        The above copyright notice and this permission notice shall be included in all
        copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
        EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
        MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
        IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
        DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
        OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
        OR OTHER DEALINGS IN THE SOFTWARE.
        """

        if not isinstance(vertices, np.ndarray):
            vertices = np.asarray(vertices, dtype=np.float32)

        if colours is not None:
            if not isinstance(colours, np.ndarray):
                colours = np.asarray(colours)
            if colours.dtype != np.uint8:
                colours = np.uint8(colours * 255)
            vertices = np.column_stack((vertices.view(dtype=np.uint32).reshape(-1, 3),
                                        colours.view(dtype=np.uint32)))

        data = vertices.tostring()

        if geom is None:
            if vertices.strides[0] == 12:
                vformat = GeomVertexFormat.get_v3()
            elif vertices.strides[0] == 16:
                vformat = GeomVertexFormat.get_v3c4()
            else:
                raise ValueError('Incompatible point clout format: {},{}'.format(vertices.dtype, vertices.shape))

            vdata = GeomVertexData('vdata', vformat, Geom.UHDynamic)
            vdata.unclean_set_num_rows(len(vertices))
            vdata.modify_array_handle(0).set_subdata(0, len(data), data)

            prim = GeomPoints(Geom.UHDynamic)
            prim.clear_vertices()
            prim.add_consecutive_vertices(0, len(vertices))
            prim.close_primitive()

            geom = Geom(vdata)
            geom.add_primitive(prim)
        else:
            vdata = geom.modify_vertex_data()
            vdata.unclean_set_num_rows(len(vertices))
            vdata.modify_array_handle(0).set_subdata(0, len(data), data)

            prim = geom.modify_primitive(0)
            prim.clear_vertices()
            prim.add_consecutive_vertices(0, len(vertices))
            prim.close_primitive()

        return geom

    def update(self, vertices, colours=None) -> None:
        if self.geom_node.get_num_geoms() == 0:
            self.geom = self.__make_points(vertices, colours)
            self.geom_node.add_geom(self.geom)
        else:
            self.geom = self.geom_node.modify_geom(0)
            self.__make_points(vertices, colours, self.geom)


if __name__ == "__main__":
    from direct.showbase.ShowBase import ShowBase
    app = ShowBase()
    cloud = PointCloud()

    NUM_POINTS = 400000

    def loop(task):
        vertices = np.random.randn(NUM_POINTS, 3).astype(np.float32)
        colours = np.ones((NUM_POINTS, 4), np.float32)
        colours[:, :3] = np.clip(np.abs(vertices), 0, 3) / 3
        cloud.update(vertices, colours)
        return task.again

    cloud.node_path.reparent_to(app.render)
    cloud.node_path.set_pos((0, 20, 0))
    app.taskMgr.doMethodLater(0.16, loop, "loop")
    app.run()
