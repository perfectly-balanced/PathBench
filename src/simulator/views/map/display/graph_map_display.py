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
    __first_frame: bool

    def __init__(self, services: Services, graph: Graph, custom_map: Map = None) -> None:
        super().__init__(services, z_index=250, custom_map=custom_map)
        self.__graph = graph

        self.edge_colour = self._services.state.views.add_colour("graph edge", RED)
        self.node_colour = self._services.state.views.add_colour("graph node", RED)

        self.__deduced_edge_colour = self.edge_colour()
        self.__deduced_node_colour = self.node_colour()

        self.__first_frame = True
        self.__graph_with_irremovable_edges_np = None

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
        if self.__graph.edges_removable:
            self.__render_lazy_edges_removable(refresh)
        else:
            self.__render_lazy_edges_irremovable(refresh)

    def __render_lazy_edges_irremovable(self, refresh: bool) -> None:
        rv = self.get_renderer_view()

        if refresh:
            def remove_child(current) -> bool:
                if self in current.aux:
                    del current.aux[self]
                return True
            
            self.__graph.walk_dfs(lambda child: remove_child(child))
            self.__first_frame = True
            if self.__graph_with_irremovable_edges_np is not None:
                self.__graph_with_irremovable_edges_np.remove_node()
                self.__graph_with_irremovable_edges_np = None

        if self.__graph_with_irremovable_edges_np is None:
            self.__graph_with_irremovable_edges_np = rv.overlay.attach_new_node('graph with irremovable edges')
        rv.push_root(self.__graph_with_irremovable_edges_np)

        if self.__first_frame: # occurs at initialisation or refresh            
            def render_child(current) -> bool:
                self.__render_child(current)
                current.aux[self] = None
                return True
            
            self.__graph.walk_dfs(lambda child: render_child(child))
            self.__first_frame = False
            rv.pop_root().flatten_strong()
            return

        for p, c in self.__graph.added:
            if self in c.aux:
                rv.draw_line(self.__deduced_edge_colour, p.position, c.position)
            else:
                self.__render_child(c)
                c.aux[self] = None
        
        rv.pop_root()

    def __render_lazy_edges_removable(self, refresh: bool) -> None:
        # node where edge added is sometimes the same node as one where an edge has been removed
        # TODO: check if it's more advantageous to keep track of whether or not should remove + redraw
        # from the beginning instead of adding edges & then remove + redraw if found to been in removed set
        """
        l = len(set([c for _, c in self.__graph.added]).intersection(set([c for _, c in self.__graph.removed])))
        print(l)
        """

        rv = self.get_renderer_view()

        if refresh:
            def remove_child(current) -> bool:
                if self in current.aux:
                    current.aux[self].remove_node()
                    del current.aux[self]
                return True
            
            self.__graph.walk_dfs(lambda child: remove_child(child))
            self.__first_frame = True

        if self.__first_frame: # occurs at initialisation or refresh            
            def render_child(current) -> bool:
                current.aux[self] = rv.push_root()
                self.__render_child(current)
                rv.pop_root().flatten_strong()
                return True
            
            self.__graph.walk_dfs(lambda child: render_child(child))
            self.__first_frame = False
            return
        
        for p, c in self.__graph.added:
            if self in c.aux:
                # overwriting geometry even
                # if its already been added instead
                # of checking is likely most optimal because
                # its rarely the case that multiple
                # edges are added to a node per frame
                # In the case of RRT* overwriting occurs quite
                # rarely, with each node requiring overwrite 
                # having between 2 - 3 edges overwritten.
                """
                if len(self.__graph.added) != len(set([c for _, c in self.__graph.added])):
                    printed = []
                    for p1, c1 in self.__graph.added:
                        if c1 in printed:
                            continue
                        cnt = 1
                        for p2, c2 in self.__graph.added:
                            if c1 == c2 and p1 != p2:
                                cnt += 1
                        if cnt != 1:
                            print(cnt)
                            printed.append(c1)
                """

                rv.push_root(c.aux[self])
                rv.draw_line(self.__deduced_edge_colour, p.position, c.position)
                rv.pop_root().flatten_strong()
            else:
                c.aux[self] = rv.push_root()
                self.__render_child(c)
                rv.pop_root().flatten_strong()
        
        # this rendering stage must be done last to
        # ensure that adding edges that were subsequently
        # removed are removed from the scene
        vs = set([c for _, c in self.__graph.removed])
        vs_no_show = []
        for v in vs:
            if self in v.aux:
                v.aux[self].remove_node()
                del v.aux[self]

            # if vertex is unreachable shouldn't add
            # The root list is quite small, commonly
            # just 2 for start and goal vertices.
            # Therefore, this implementation should
            # be quite fast. It's unlikely that a 
            # large tree will suddenly be removed

            if v in vs_no_show:
                continue

            if not v.parents and v not in self.__graph.root_vertices:
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

            v.aux[self] = rv.push_root()
            self.__render_child(v)
            rv.pop_root().flatten_strong()

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
        if self._map.size.n_dim == 2:
            rv.draw_circle_filled(current.position, colour=self.__deduced_node_colour)
        else:
            rv.draw_sphere(current.position, colour=self.__deduced_node_colour)

    def get_tracked_data(self) -> List[Tracked]:
        return [self.__graph] if isinstance(self.__graph, Tracked) else []
