from typing import List, Dict, TYPE_CHECKING

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import DynamicColour, Colour, RED
from structures.tracked import Tracked
from algorithms.classic.sample_based.core.graph import TrackedGraph, Graph, Vertex

from panda3d.core import NodePath


class GraphMapDisplay(MapDisplay):
    __graph: Graph

    edge_colour: DynamicColour
    node_colour: DynamicColour
    __deduced_edge_colour: Colour
    __deduced_node_colour: Colour

    __v_to_np: Dict[Vertex, NodePath]

    def __init__(self, services: Services, graph: Graph, custom_map: Map = None) -> None:
        super().__init__(services, z_index=250, custom_map=custom_map)
        self.__graph = graph

        self.edge_colour = self._services.state.add_colour("graph edge", RED)
        self.node_colour = self._services.state.add_colour("graph node", RED)

        self.__deduced_edge_colour = self.edge_colour()
        self.__deduced_node_colour = self.node_colour()

        self.__v_to_np = {}

    def render(self, refresh: bool) -> bool:
        if not super().render(refresh):
            return False

        c = self.edge_colour()
        refresh = refresh or c != self.__deduced_edge_colour
        self.__deduced_edge_colour = c

        c = self.node_colour()
        refresh = refresh or c != self.__deduced_node_colour
        self.__deduced_node_colour = c

        if not refresh and isinstance(self.__graph, TrackedGraph):
            self.__render_lazy()
        else:
            for np in self.__v_to_np:
                np.remove_node()
            self.__v_to_np.clear()
            self.__render_eager()
        return True

    def __render_lazy(self) -> None:
        # node where edge added is (virtually) never the same node as one where an edge has been removed
        # hence it's more advantageous to add edges & then remove + redraw if found to been in removed set
        # instead of keeping track of whether or not should remove + redraw from the beginning
        # l = len(set([c for _, c in self.__graph.added]).intersection(set([c for _, c in self.__graph.removed])))
        # if l > 0:
        #    print(l)

        rv = self.get_renderer_view()

        for p, c in self.__graph.added:
            if c in self.__v_to_np:
                # overwriting geometry even
                # if its already been added instead
                # of checking is most optimal because
                # its rarely the case that multiple
                # edges are added to a node per frame
                # In the case of RRT overwriting never
                # occurs.
                # print(len(self.__graph.added) == len(set([c for _, c in self.__graph.added])))

                np = self.__v_to_np[c]
                rv.start_collecting_nodes(np)
                rv.draw_line(self.__deduced_edge_colour, p.position, c.position)
                rv.end_collecting_nodes()
            else:
                rv.start_collecting_nodes()
                self.__render_child(c)
                np = rv.end_collecting_nodes()
                self.__v_to_np[c] = np
        
        vs = set([c for _, c in self.__graph.removed])
        for v in vs:
            # todo if vertex is unreachable shouldn't add
            # should be able to do this quite fast. By, for example
            # checking if a node doesn't have parents. If it doesn't
            # then all its children (recursively) should no longer be
            # rendered if they only have unique parent.
            # If any of these nodes are root nodes then this overrides
            # discarding the node. The root list is quite small.
            # Commonly just 2 for start and goal vertices.
            
            if v in self.__v_to_np:
                self.__v_to_np[v].remove_node()

            rv.start_collecting_nodes()
            self.__render_child(v)
            np = rv.end_collecting_nodes()

            self.__v_to_np[v] = np

    def __render_eager(self) -> None:
        self.__graph.walk_dfs(lambda child: self.__render_child(child))

    def __render_child(self, current: Vertex) -> bool:
        self.__render_node(current)
        if len(current.parents) != 0:
            self.__render_edge(current)
        return True

    def __render_edge(self, current: Vertex):
        rv = self.get_renderer_view()
        for parent in current.parents:
            rv.draw_line(self.__deduced_edge_colour, current.position, parent.position)

    def __render_node(self, current: Vertex) -> None:
        rv = self.get_renderer_view()
        if self._map.size.n_dim == 3:
            rv.draw_circle_filled(current.position, colour=self.__deduced_node_colour)
        else:
            rv.draw_sphere(current.position, colour=self.__deduced_node_colour)

    def get_tracked_data(self) -> List[Tracked]:
        return [self.__graph] if isinstance(self.__graph, Tracked) else []
