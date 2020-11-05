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

    def render(self, refresh: bool) -> None:
        c = self.edge_colour()
        refresh = refresh or c != self.__deduced_edge_colour
        self.__deduced_edge_colour = c

        c = self.node_colour()
        refresh = refresh or c != self.__deduced_node_colour
        self.__deduced_node_colour = c

        if isinstance(self.__graph, TrackedGraph):
            self.__render_lazy(refresh)
        else:
            self.__render_eager()

    def __render_lazy(self, refresh: bool) -> None:
        # node where edge added is (virtually) never the same node as one where an edge has been removed
        # hence it's more advantageous to add edges & then remove + redraw if found to been in removed set
        # instead of keeping track of whether or not should remove + redraw from the beginning
        # l = len(set([c for _, c in self.__graph.added]).intersection(set([c for _, c in self.__graph.removed])))
        # if l > 0:
        #    print(l)

        rv = self.get_renderer_view()

        if refresh:
            for np in self.__v_to_np.items():
                np.remove_node()
            self.__v_to_np.clear()

        if not self.__v_to_np: # occurs at initialisation or refresh            
            def render_child(current) -> bool:
                rv.start_collecting_nodes()
                self.__render_child(current)
                np = rv.end_collecting_nodes()
                self.__v_to_np[current] = np
                return True
            
            self.__graph.walk_dfs(lambda child: render_child(child))
            return

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
        
        # this rendering stage must be done last to
        # ensure that adding edges that were subsequently
        # removed are removed from the scene
        vs = set([c for _, c in self.__graph.removed])
        vs_no_show = []
        for v in vs:
            if v in self.__v_to_np:
                self.__v_to_np[v].remove_node()

            # if vertex is unreachable shouldn't add
            # The root list is quite small, commonly
            # just 2 for start and goal vertices.
            # Therefore, this implementation should
            # be quite fast. It's unlikely that a 
            # large tree will suddenly be removed

            if v in vs_no_show:
                continue

            if v.parents.empty() and v not in self.__graph.root_vertices:
                vs_no_show.append(v)
                def check_no_show(current):
                    for p in current.parents:
                        if p not in vs_no_show:
                            return
                    vs_no_show.append(current)
                    for c in current.children:
                        check_no_show(c)
                for c in v.children:
                    check_no_show(c)
                continue

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
