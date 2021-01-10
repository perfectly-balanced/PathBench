
import math
import os
import json
from itertools import product
from typing import Tuple, List, TYPE_CHECKING, Dict, Callable, Type, Set, Any, Optional

import torch
import numpy as np
from matplotlib import pyplot as plt
from natsort import natsorted

from algorithms.lstm.LSTM_CAE_tile_by_tile import CAE
from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.testing.a_star_testing import AStarTesting
from algorithms.configuration.configuration import Configuration
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.debug import DebugLevel
from simulator.services.resources.atlas import Atlas
from simulator.services.services import Services
from simulator.simulator import Simulator
from structures import Point, Size
from utility.constants import DATA_PATH
from utility.progress import Progress
from utility.timer import Timer

if TYPE_CHECKING:
    from main import MainRunner


class Generator:
    """
    Used to generate maps
    """
    __services: Services
    COLOR_EPSILON: int = 230
    GOAL_COLOR: Tuple[int, int, int, int] = (0, 255, 0, 255)
    AGENT_COLOR: Tuple[int, int, int, int] = (255, 0, 0, 255)
    WALL_COLOR: Tuple[int, int, int, int] = (0, 0, 0, 255)

    AVAILABLE_GENERATION_METHODS: Set["str"] = {"uniform_random_fill", "block_map", "house"}

    def __init__(self, services: Services) -> None:
        self.__services = services
        self.generate_map_from_image = self.__services.debug.debug_func(DebugLevel.BASIC)(self.generate_map_from_image)
        self.generate_maps = self.__services.debug.debug_func(DebugLevel.BASIC)(self.generate_maps)
        self.label_maps = self.__services.debug.debug_func(DebugLevel.BASIC)(self.label_maps)

    def generate_map_from_image(self, image_name: str, rand_entities: bool = False, entity_radius: int = None, house_expo_flag: bool = False) -> Map:
        """
        Generate a map from an image
        Load the image from the default location and save the map in the default location
        :param image_name: The image name
        :return: The map
        """
        self.__services.debug.write("Started map generation from image: " + str(image_name) + " With House_expo = " + str(house_expo_flag),
                                    DebugLevel.BASIC)
        timer: Timer = Timer()

        # loading image
        if house_expo_flag:
            surface: np.ndarray = self.__services.resources.house_expo_dir.load(image_name)
        else:
            surface: np.ndarray = self.__services.resources.images_dir.load(image_name)
        height, width, _ = surface.shape
        self.__services.debug.write("Image loaded with Resolution:" + str(width) + " x " + str(height), DebugLevel.HIGH)

        grid = np.full(surface.shape[:-1], Map.CLEAR_ID, dtype=np.uint8)
        agent_avg_location: np.ndarray = np.array([.0, .0])
        agent_avg_count: int = 1
        goal_avg_location: np.ndarray = np.array([.0, .0])

        if house_expo_flag:
            '''
            We can optimize for house_expo dataset by skipping the check for the goal and agent at each pixel,
            instead, we can only identify obstacles
            '''
            self.__services.debug.write("Begin iteration through map", DebugLevel.HIGH)
            for idx in np.ndindex(surface.shape[:-1]):
                if Generator.is_in_color_range(surface[idx], Generator.WALL_COLOR):
                    grid[idx] = DenseMap.WALL_ID
        else:
            for idx in np.ndindex(surface.shape[:-1]):
                if Generator.is_in_color_range(surface[idx], Generator.AGENT_COLOR, 5):
                    agent_avg_location, agent_avg_count = \
                        Generator.increment_moving_average(agent_avg_location, agent_avg_count, np.array(idx[::-1]))
                elif Generator.is_in_color_range(surface[idx], Generator.GOAL_COLOR, 5):
                    goal_avg_location, goal_avg_count = \
                        Generator.increment_moving_average(goal_avg_location, goal_avg_count, np.array(idx[::-1]))
                if Generator.is_in_color_range(surface[idx], Generator.WALL_COLOR):
                    grid[idx] = DenseMap.WALL_ID

        agent_avg_location = np.array(agent_avg_location, dtype=int)
        goal_avg_location = np.array(goal_avg_location, dtype=int)
        agent_radius: float = 0

        if rand_entities:
            self.__place_random_agent_and_goal(grid, Size(height, width))
            self.__services.debug.write("Placed random agent and goal ", DebugLevel.HIGH)
        else:
            grid[tuple(agent_avg_location[::-1])] = DenseMap.AGENT_ID
            grid[tuple(goal_avg_location[::-1])] = DenseMap.GOAL_ID

        if not house_expo_flag:
            '''
            We can optimize the house_expo generation by skipping this step, 
            as we have already defined the agent radius
            '''
            self.__services.debug.write("Skipped agent_radius change checking ", DebugLevel.HIGH)

            for idx in np.ndindex(surface.shape[:-1]):
                if Generator.is_in_color_range(surface[idx], Generator.AGENT_COLOR, 5):
                    '''
                    If color at x y is red (agent) then change the radius of the agent to the max 
                    Change the agent radius to the max between the old radius (from previous iteration )
                    and the magnitude of the agent location - the point 
                        and the magnitude of the agent location - the point 
                    and the magnitude of the agent location - the point 
                    This basically defines the agent radius as the largest red size. But we don't need to do
                    this as we are supplying our own radius
                    '''
                    agent_radius = max(agent_radius, np.linalg.norm(agent_avg_location - np.array(idx[::-1])))

        agent_radius = int(agent_radius)

        if entity_radius:
            agent_radius = entity_radius

        res_map: DenseMap = DenseMap(grid)
        res_map.agent.radius = agent_radius
        res_map.goal.radius = agent_radius

        self.__services.debug.write("Generated initial dense map in " + str(timer.stop()) + " seconds",
                                    DebugLevel.BASIC)
        timer = Timer()
        res_map.extend_walls()
        self.__services.debug.write("Extended walls in " + str(timer.stop()) + " seconds", DebugLevel.BASIC)
        map_name: str = str(image_name.split('.')[0]) + ".pickle"
        if house_expo_flag:
            path = os.path.join(os.path.join(DATA_PATH, "maps"), "house_expo")
            self.__services.resources.house_expo_dir.save(map_name, res_map, path)
        else:
            self.__services.resources.maps_dir.save(map_name, res_map)
        self.__services.debug.write("Finished generation. Map is in resources folder", DebugLevel.BASIC)
        return res_map

    def __in_bounds(self, pt: Point, dimensions: Size) -> bool:
        z = 0 <= pt.z < dimensions.depth if dimensions.n_dim == 3 else True
        return 0 <= pt.x < dimensions.width and 0 <= pt.y < dimensions.height and z

    def __get_rand_position(self, dimensions: Size, start: Optional[Point] = None) -> Point:
        if start is None:
            start = Point(*([0] * dimensions.n_dim))

        return Point(*np.random.randint([*start], [*dimensions]))

    def __generate_random_map(self, dimensions: Size, obstacle_fill_rate: float = 0.3) -> Map:
        grid: np.array = np.zeros(dimensions)
        fill: float = math.prod(dimensions) * obstacle_fill_rate
        nr_of_obstacles = 0

        while nr_of_obstacles < fill:
            obst_pos: Point = self.__get_rand_position(dimensions)

            if grid[obst_pos.values] == DenseMap.CLEAR_ID:
                grid[obst_pos.values] = DenseMap.WALL_ID
                nr_of_obstacles += 1

        self.__place_random_agent_and_goal(grid, dimensions)

        return DenseMap(grid)

    def __place_random_agent_and_goal(self, grid: np.array, dimensions: Size):
        def randomly_place(cell_id):
            nonlocal grid

            MAX_ITERATIONS: Final[int] = 10000
            for _ in range(MAX_ITERATIONS):
                p: Point = self.__get_rand_position(dimensions)

                if grid[p.values] == Map.CLEAR_ID:
                    grid[p.values] = cell_id
                    return

            for idx in np.ndindex(grid.shape):
                if grid[p.values] == Map.CLEAR_ID:
                    grid[p.values] = cell_id
                    return

            raise Exception("Map doesn't contain any traversable cells")

        randomly_place(Map.AGENT_ID)
        randomly_place(Map.GOAL_ID)

    def __place_entity_near_corner(self, entity: Type[Entity], corner: int, grid: List[List[int]], dimensions: Size) -> \
            List[List[int]]:
        for _ in range(corner):
            grid = self.__rotate_by_90(grid, dimensions)

        token = DenseMap.AGENT_ID if entity == Agent else DenseMap.GOAL_ID

        for l in range(min(dimensions.width, dimensions.height)):
            should_break = False
            for y in range(l + 1):
                if grid[y][l] == DenseMap.CLEAR_ID:
                    grid[y][l] = token
                    should_break = True
                    break

            if should_break:
                break

            for x in range(l + 1):
                if grid[l][x] == DenseMap.CLEAR_ID:
                    grid[x][l] = token
                    should_break = True
                    break

            if should_break:
                break

        for _ in range(4 - corner):
            grid = self.__rotate_by_90(grid, dimensions)

        return grid

    def __rotate_by_90(self, grid, dimensions):
        res: List[List[int]] = [[0 for _ in range(dimensions.height)] for _ in range(dimensions.width)]
        for i in range(len(res)):
            for j in range(len(res[i])):
                res[i][j] = grid[len(grid) - j - 1][i]
        return res

    def __generate_random_const_obstacles(self, dimensions: Size, obstacle_fill_rate: float, nr_of_obstacles: int):
        grid: np.array = np.zeros(dimensions)
        total_fill = int(math.prod(dimensions) * obstacle_fill_rate)
        print(math.prod(dimensions))
        for i in range(nr_of_obstacles):
            next_obst_fill = np.random.randint(total_fill)

            if i == nr_of_obstacles - 1:
                next_obst_fill = total_fill

            if next_obst_fill == 0:
                break

            while True:
                if(dimensions.n_dim == 3):
                    first_side = np.random.randint(round(next_obst_fill ** (1. / 3))) + 1
                    second_side = np.random.randint(round(next_obst_fill ** (1. / 3))) + 1
                    third_side = np.random.randint(round(next_obst_fill ** (1. / 3))) + 1
                    size = Size(first_side, second_side, third_side)
                else:
                    first_side = np.random.randint(round(next_obst_fill ** (1. / 2))) + 1
                    second_side = max(int(next_obst_fill / first_side), 1)
                    size = Size(first_side, second_side)

                top_left_corner = self.__get_rand_position(dimensions)

                if self.__can_place_square(size, top_left_corner, dimensions):
                    self.__place_square(grid, size, top_left_corner)
                    break

            total_fill -= next_obst_fill

        self.__place_random_agent_and_goal(grid, dimensions)
        return DenseMap(grid)

    def __place_square(self, grid: np.array, size: Size, top_left_corner: Point) -> None:
        for index in np.ndindex(*size):
            p = tuple(np.add(index, top_left_corner.values))
            grid[p] = DenseMap.WALL_ID

    def __can_place_square(self, size: Size, top_left_corner: Point, dimensions: Size) -> bool:
        for index in np.ndindex(*([2]*size.n_dim)):
            d_add = [x * y for (x, y) in zip([*index], [*size])]
            p = Point(*(np.add(top_left_corner.values, d_add)))
            if not self.__in_bounds(p, dimensions):
                return False

        return True

    def __generate_random_house(self, dimensions: Size, min_room_size: Size = Size(10, 10),
                                max_room_size: Size = Size(40, 40), door_size: int = 2) -> Map:
        grid: np.array = np.zeros(dimensions)
        rooms: List[Tuple[Point, Size]] = []

        def in_bounds(pos) -> bool:
            return all([pos[i] >= 0 and pos[i] < grid.shape[i] for i in range(len(grid.shape))])

        def __place_room(grid: np.array, size: Size, top_left_corner: Point) -> None:
            size_exc = tuple(x - 1 for x in size)
            for index in np.ndindex(*size_exc):
                p = [elem + top_left_corner[i] for i, elem in enumerate(index)]
                if in_bounds(Point(*p)):
                    grid[tuple(p)] = DenseMap.CLEAR_ID
                for i in range(len(size)):
                    if p[i] == top_left_corner[i] or p[i] == top_left_corner[i] + size[i] - 1:
                        if in_bounds(Point(*p)):
                            grid[tuple(p)] = DenseMap.WALL_ID

            rooms.append((top_left_corner, size))

        def __get_subdivision_number(dim, i: int) -> int:
            min_size = min_room_size[i]

            dim_size = dim[i]

            total_split = dim_size - 2 * min_size + 1
            rand_split = np.random.randint(0, total_split + 1)
            return min_size + rand_split

        def __subdivide(top_left_corner, dim) -> None:
            if all([dim[i] < 2 * min_room_size[i] - 1 for i in range(len(dim))]):
                __place_room(grid, dim, top_left_corner)
                return
            is_vertical_split = int(np.random.randint(0, len(dim)))
            split = [0] * len(dim)
            for i in range(len(dim)):
                if dim[i] < 2 * min_room_size[i] - 1:
                    split[i] = 1
                    is_vertical_split = (1 + i) % (len(dim) - 1)

            while(split[is_vertical_split] == 1):
                is_vertical_split = int(np.random.randint(0, len(dim)))

            if all([dim[i] <= max_room_size[i] for i in range(len(dim))]):
                # do we want a 50-50 chance here? (or should 2 be len(dim)?)
                if int(np.random.randint(0, 2)) == 0:
                    __place_room(grid, dim, top_left_corner)
                    return

            # split
            new_top_left_corner1 = top_left_corner

            subdiv = __get_subdivision_number(dim, is_vertical_split)
            new_dim1 = list(dim)
            new_dim1[is_vertical_split] = subdiv

            new_top_left_corner2 = list(top_left_corner)
            new_top_left_corner2[is_vertical_split] = top_left_corner[is_vertical_split] + new_dim1[is_vertical_split] - 1

            new_dim2 = list(dim)
            new_dim2[is_vertical_split] = dim[is_vertical_split] - new_dim1[is_vertical_split] + 1

            __subdivide(Point(*new_top_left_corner1), tuple(new_dim1))
            __subdivide(Point(*new_top_left_corner2), tuple(new_dim2))

        # place rooms

        __subdivide(Point(*([-1] * len(dimensions))), Size(*[dimensions[i] + 2 for i in range(len(dimensions))]))

        def __get_nearby_rooms_edges(room) -> (list, list):
            room_top_left_point, room_size = room
            edges = []
            full_edge = []
            q = 0

            if(len(room_size) == 2):

                for x in range(room_top_left_point[0], room_top_left_point[0] + room_size[0]):
                    y = room_top_left_point[1]
                    # up
                    if in_bounds(Point(x, y)):
                        full_edge.append(Point(x, y))
                        n_x, n_y = x, y - 1
                        if in_bounds(Point(n_x, n_y)) and grid[n_x][n_y] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

                for y in range(room_top_left_point[1], room_top_left_point[1] + room_size[1]):

                    # right
                    x = room_top_left_point[0] + room_size[0] - 1
                    if in_bounds(Point(x, y)):
                        full_edge.append(Point(x, y))
                        n_x, n_y = x + 1, y
                        if in_bounds(Point(n_x, n_y)) and grid[n_x][n_y] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

                for x in range(room_top_left_point[0], room_top_left_point[0] + room_size[0]):

                    # bottom
                    y = room_top_left_point[1] + room_size[1] - 1
                    if in_bounds(Point(x, y)):
                        full_edge.append(Point(x, y))
                        n_x, n_y = x, y + 1
                        if in_bounds(Point(n_x, n_y)) and grid[n_x][n_y] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

                for y in range(room_top_left_point[1], room_top_left_point[1] + room_size[1]):
                    x = room_top_left_point[0]

                    # left
                    if in_bounds(Point(x, y)):
                        full_edge.append(Point(x, y))
                        n_x, n_y = x - 1, y
                        if in_bounds(Point(n_x, n_y)) and grid[n_x][n_y] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

            elif len(room_size) == 3:

                for index in product([room_top_left_point[0], room_top_left_point[0] + room_size[0] - 1],
                                     [room_top_left_point[1] + i for i in range(room_size[1])],
                                     [room_top_left_point[2] + i for i in range(room_size[2])]):
                    if in_bounds(Point(*index)):
                        full_edge.append(Point(*index))
                        n_y, n_z = index[1], index[2]
                        if index[0] == room_top_left_point[0]:
                            n_x = index[0] - 1
                        else:
                            n_x = index[0] + 1

                        if in_bounds(Point(n_x, n_y, n_z)) and grid[(n_x, n_y, n_z)] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

                for index in product([room_top_left_point[0] + i for i in range(room_size[0])],
                                     [room_top_left_point[1], room_top_left_point[1] + room_size[1] - 1],
                                     [room_top_left_point[2] + i for i in range(room_size[2])]):
                    if in_bounds(Point(*index)):
                        full_edge.append(Point(*index))
                        n_x, n_z = index[0], index[2]
                        if index[1] == room_top_left_point[1]:
                            n_y = index[1] - 1
                        else:
                            n_y = index[1] + 1

                        if in_bounds(Point(n_x, n_y, n_z)) and grid[(n_x, n_y, n_z)] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

                for index in product([room_top_left_point[0] + i for i in range(room_size[0])],
                                     [room_top_left_point[1] + i for i in range(room_size[1])],
                                     [room_top_left_point[2], room_top_left_point[2] + room_size[2] - 1]):
                    if in_bounds(Point(*index)):
                        full_edge.append(Point(*index))
                        n_x, n_y = index[0], index[1]
                        if index[2] == room_top_left_point[2]:
                            n_z = index[2] - 1
                        else:
                            n_z = index[2] + 1

                        if in_bounds(Point(n_x, n_y, n_z)) and grid[(n_x, n_y, n_z)] == DenseMap.WALL_ID:
                            edges.append(q)
                        q += 1

            return edges, full_edge

        doors = set()

        # place doors
        for room in rooms:
            neighbours, full_edge = __get_nearby_rooms_edges(room)

            if len(full_edge) <= door_size + 2:
                continue

            for i in range(len(neighbours)):
                nxt = (i + 1) % len(neighbours)

                if abs(neighbours[i] - neighbours[nxt]) >= door_size + 1 or len(neighbours) == 1:
                    found_door = False
                    # check if no doors are placed
                    for q in range(
                            len(full_edge) if len(neighbours) == 1 else abs(neighbours[i] - neighbours[nxt] + 1)):
                        if full_edge[(neighbours[i] + q) % len(full_edge)] in doors:
                            found_door = True
                            break

                    if found_door:
                        continue

                    # place door

                    if int(np.random.randint(0, 4)) == 0:
                        continue

                    diff = abs(neighbours[i] - neighbours[nxt]) - door_size + 1

                    if len(neighbours) == 1:
                        diff = len(full_edge)

                    start = int(np.random.randint(1, diff))
                    next_door = neighbours[i] + start
                    for q in range(door_size):
                        pt = full_edge[(next_door + q) % len(full_edge)]
                        grid[pt.values] = DenseMap.CLEAR_ID
                        doors.add(pt)

        self.__place_random_agent_and_goal(grid, dimensions)

        return DenseMap(grid)

    def generate_maps(self, nr_of_samples: int, dimensions: Size, gen_type: str, fill_range: List[float],
                      nr_of_obstacle_range: List[int], min_map_range: List[int], max_map_range: List[int], num_dim: int = 2, json_save: bool = False) -> List[Map]:
        if gen_type not in Generator.AVAILABLE_GENERATION_METHODS:
            raise Exception("Generation type {} does not exist in {}".format(gen_type, self.AVAILABLE_GENERATION_METHODS))

        if nr_of_samples <= 0:
            return []

        self.__services.debug.write("""Starting Generation: [
            nr_of_samples: {},
            gen_type: {},
            dimensions: {},
            fill_range: {},
            nr_of_obstacle_range: {},
            min_map_range: {},
            max_map_range: {}
        ] 
        """.format(nr_of_samples, gen_type, dimensions, fill_range, nr_of_obstacle_range, min_map_range, max_map_range), DebugLevel.BASIC)

        atlas_name = "{}_{}".format(gen_type, str(nr_of_samples))
        atlas: Atlas = self.__services.resources.maps_dir.create_atlas(atlas_name)
        progress_bar: Progress = self.__services.debug.progress_debug(nr_of_samples, DebugLevel.BASIC)
        progress_bar.start()
        maps: List[Map] = []

        for _ in range(nr_of_samples):
            fill_rate = fill_range[0] + np.random.rand() * (fill_range[1] - fill_range[0])
            if gen_type == "uniform_random_fill":  # random fill
                mp: Map = self.__generate_random_map(dimensions, fill_rate)
            elif gen_type == "block_map":  # block
                mp: Map = self.__generate_random_const_obstacles(
                    dimensions,
                    fill_rate,
                    np.random.randint(nr_of_obstacle_range[0], nr_of_obstacle_range[1])
                )
            else:  # house map
                min_map_size = int(np.random.randint(min_map_range[0], min_map_range[1]))
                max_map_size = int(np.random.randint(max_map_range[0], max_map_range[1]))
                mp: Map = self.__generate_random_house(
                    dimensions,
                    min_room_size=Size(*([min_map_size] * dimensions.n_dim)),
                    max_room_size=Size(*([max_map_size] * dimensions.n_dim)),
                )
            atlas.append(mp)
            maps.append(mp)
            progress_bar.step()

            map_as_dict = {
                "type": "DenseMap",
                "version": 1,
                "goal": [*mp.goal.position.values],
                "agent": [*mp.agent.position.values],
                "grid": mp.grid.tolist()
            }
            dimensions_path = '_3d' if num_dim == 3 else ''
            if json_save:
                with open(self.__services.resources.maps_dir._full_path() + atlas_name + dimensions_path + '/' + str(_) + dimensions_path + '.json', 'w') as outfile:
                    json.dump(map_as_dict, outfile)
                    self.__services.debug.write("Dumping JSON: " + str(_) + "\n", DebugLevel.LOW)

        # print(maps[1].grid)

        self.__services.debug.write("Finished atlas generation: " + str(atlas_name) + "\n", DebugLevel.BASIC)
        return maps

    def label_maps(self, atlases: List[str], feature_list: List[str], label_list: List[str],
                   single_feature_list: List[str], single_label_list: List[str]) -> None:
        if not atlases:
            return

        self.__services.debug.write("""Starting Labelling: [
            atlases: {},
            feature_list: {},


            label_list: {},
            single_feature_list: {},
            single_label_list: {}
        ] 
        """.format(atlases, feature_list, label_list, single_feature_list, single_label_list), DebugLevel.BASIC)

        label_atlas_name = "training_" + "_".join(atlases)

        # special case where we only have 1 atlas, overwrite is True by default
        if len(atlases) == 1:
            self.__services.debug.write("Processing single atlas (overwrite True)", DebugLevel.BASIC)
            res = self.__label_single_maps(atlases[0], feature_list, label_list, single_label_list, single_label_list,
                                           True)
            self.__save_training_data(label_atlas_name, res)
            return

        # more atlases

        t: List[Dict[str, any]] = []

        for name in atlases:
            training_atlas = "training_" + name
            self.__services.debug.write("Processing " + str(training_atlas), DebugLevel.BASIC)
            next_res: List[Dict[str, any]] = self.__label_single_maps(name, feature_list, label_list, single_label_list,
                                                                      single_label_list, False)

            # save if it does not exist
            if not self.__services.resources.training_data_dir.exists(training_atlas, ".pickle"):
                self.__save_training_data(training_atlas, next_res)
            t = t + next_res

        self.__save_training_data(label_atlas_name, t)

    def __save_training_data(self, training_name: str, training_data: List[Dict[str, Any]]) -> None:
        self.__services.debug.write("Saving atlas labelling: " + training_name, DebugLevel.BASIC)
        self.__services.resources.training_data_dir.save(training_name, training_data)
        self.__services.debug.write("Finished atlas labelling: " + training_name + "\n", DebugLevel.BASIC)

    def __label_single_maps(self, atlas_name, feature_list: List[str], label_list: List[str],
                            single_feature_list: List[str], single_label_list: List[str], overwrite: bool) -> List[
            Dict[str, any]]:
        """
        Passed atlas name, feature list, label list, and returns res object with the map features labelled for training
        """
        if not atlas_name:
            return []

        if not overwrite and self.__services.resources.training_data_dir.exists("training_" + atlas_name, ".pickle"):
            self.__services.debug.write("Found in training data. Loading from training data", DebugLevel.BASIC)
            return self.__services.resources.training_data_dir.load("training_" + atlas_name)

        self.__services.debug.write("Loading maps", DebugLevel.BASIC)
        maps: List[Map] = self.__services.resources.maps_dir.get_atlas(atlas_name).load_all()

        res: List[Dict[str, any]] = []

        progress_bar: Progress = self.__services.debug.progress_debug(len(maps), DebugLevel.BASIC)
        progress_bar.start()

        # process atlas
        for m in maps:
            config = Configuration()
            config.simulator_algorithm_type = AStar
            config.simulator_testing_type = AStarTesting
            config.simulator_initial_map = m
            services: Services = Services(config)
            simulator: Simulator = Simulator(services)
            testing: AStarTesting = simulator.start()

            features: Dict[str, any] = {}
            arg: str
            for arg in ["map_obstacles_percentage",
                        "goal_found",
                        "distance_to_goal",
                        "original_distance_to_goal",
                        "trace",
                        "total_steps",
                        "total_distance",
                        "smoothness_of_trajectory",
                        "total_time",
                        "algorithm_type",
                        "fringe",
                        "search_space"
                        ]:
                features[arg] = testing.get_results()[arg]

            features["features"] = MapProcessing.get_sequential_features(testing.map, feature_list)
            features["labels"] = MapProcessing.get_sequential_labels(testing.map, label_list)
            features["single_features"] = MapProcessing.get_single_features(m, single_feature_list)
            features["single_labels"] = MapProcessing.get_single_labels(m, single_label_list)
            res.append(features)
            progress_bar.step()

        return res

    def augment_label_maps(self, atlases: List[str], feature_list: List[str], label_list: List[str],
                           single_feature_list: List[str], single_label_list: List[str]) -> None:
        if not atlases:
            return

        self.__services.debug.write("""Starting Augmentation: [
            atlases: {},
            feature_list: {},
            label_list: {},
            single_feature_list: {},
            single_label_list: {}
        ] 
        """.format(atlases, feature_list, label_list, single_feature_list, single_label_list), DebugLevel.BASIC)

        label_atlas_name = "training_" + "_".join(atlases)

        self.__services.debug.write("Loading maps", DebugLevel.BASIC)
        maps: List[Map] = []
        for name in atlases:
            maps = maps + self.__services.resources.maps_dir.get_atlas(name).load_all()

        self.__services.debug.write("Loading atlas", DebugLevel.BASIC)
        t: List[Dict[str, any]] = self.__services.resources.training_data_dir.load(label_atlas_name)

        progress_bar: Progress = self.__services.debug.progress_debug(len(t), DebugLevel.BASIC)
        progress_bar.start()

        for i in range(len(t)):
            config = Configuration()
            config.simulator_algorithm_type = AStar
            config.simulator_testing_type = AStarTesting
            config.simulator_initial_map = maps[i]
            services: Services = Services(config)
            simulator: Simulator = Simulator(services)
            testing: AStarTesting = simulator.start()

            if feature_list:
                seq_features = MapProcessing.get_sequential_features(testing.map, feature_list)
                for q in range(len(t[i]["features"])):
                    t[i]["features"][q].update(seq_features[q])

            if label_list:
                seq_labels = MapProcessing.get_sequential_labels(testing.map, label_list)
                for q in range(len(t[i]["labels"])):
                    t[i]["labels"][q].update(seq_labels[q])

            if single_feature_list:
                t[i]["single_features"].update(MapProcessing.get_single_features(maps[i], single_feature_list))

            if single_label_list:
                t[i]["single_labels"].update(MapProcessing.get_single_labels(maps[i], single_label_list))
            progress_bar.step()

        self.__services.debug.write("Saving atlas augmentation: " + str(label_atlas_name), DebugLevel.BASIC)
        self.__services.resources.training_data_dir.save(label_atlas_name, t)
        self.__services.debug.write("Finished atlas augmentation: " + str(label_atlas_name) + "\n", DebugLevel.BASIC)

    def modify_map(self, map_name: str, modify_f: Callable[[Map], Map]) -> None:
        mp: Map = self.__services.resources.maps_dir.load(map_name)
        mp = modify_f(mp)
        self.__services.resources.maps_dir.save(map_name, mp)

    def convert_house_expo(self):
        path = os.path.join(os.path.join(DATA_PATH, "maps"), "house_expo")
        print("Taking images from" + path)

        for filename in natsorted(os.listdir(path)):
            print('filename:', filename)
            self.generate_map_from_image(filename, True, 2, True)

    @staticmethod
    def increment_moving_average(cur_value: np.ndarray, count: int, new_number: np.ndarray) -> Tuple[np.ndarray, int]:
        """
        Increments the average using online average update
        :param cur_value: The current value of the moving average
        :param count: The number of elements in the average
        :param new_number: The new number that needs to be added to the average
        :return: The new average along with the new number of elements
        """
        cur_value += (new_number - cur_value) / count
        return cur_value, count + 1

    @staticmethod
    def is_in_color_range(actual_color: Tuple[int, int, int, int], search_color: Tuple[int, int, int, int], eps: float = None) -> bool:
        """
        Checks if the colors are close enough to each other
        :param actual_color: The actual color
        :param search_color: The other color
        :return: The result
        """

        if not eps:
            eps = Generator.COLOR_EPSILON

        return np.linalg.norm(np.array(actual_color, dtype=float) - np.array(search_color, dtype=float)) < eps

    @staticmethod
    def main(m: 'MainRunner') -> None:
        generator: Generator = Generator(m.main_services)
        settings = m.main_services.settings

        if settings.generator_modify:
            generator.modify_map(*settings.generator_modify())

        fill_rate = [settings.generator_obstacle_fill_min, settings.generator_obstacle_fill_max]
        min_room_size = max(settings.generator_min_room_size, settings.generator_size)
        max_room_size = min(settings.generator_max_room_size, settings.generator_size)
        min_room = [min_room_size, min_room_size + 1]
        max_room = [max_room_size, max_room_size + 1]

        if not settings.generator_house_expo:
            if settings.generator_size == 8:  # Fill rate and nr obstacle range (1,2) is for unifrom random fill (0.1,0.2)
                maps = generator.generate_maps(settings.generator_nr_of_examples, Size(*([8] * settings.num_dim)),
                                               settings.generator_gen_type, fill_rate, [1, 2], min_room, max_room, num_dim=settings.num_dim, json_save=True)

            elif settings.generator_size == 16:
                maps = generator.generate_maps(settings.generator_nr_of_examples, Size(*([16] * settings.num_dim)),
                                               settings.generator_gen_type, fill_rate, [2, 4], min_room, max_room, num_dim=settings.num_dim, json_save=True)

            elif settings.generator_size == 28:
                maps = generator.generate_maps(settings.generator_nr_of_examples, Size(*([28] * settings.num_dim)),
                                               settings.generator_gen_type, fill_rate, [1, 4], min_room, max_room, num_dim=settings.num_dim, json_save=True)

            else:
                maps = generator.generate_maps(settings.generator_nr_of_examples, Size(*([64] * settings.num_dim)),
                                               settings.generator_gen_type, fill_rate, [1, 6], min_room, max_room, num_dim=settings.num_dim, json_save=False)

            # This will display 5 of the maps generated
            if settings.generator_show_gen_sample:
                if settings.generator_nr_of_examples > 0:
                    # show sample
                    for i in range(5):
                        plt.imshow(maps[i].grid, cmap=CAE.MAP_COLORMAP_FULL)
                        plt.show()

        if settings.generator_aug_labelling_features or \
           settings.generator_aug_labelling_labels or \
           settings.generator_aug_single_labelling_features or \
           settings.generator_aug_single_labelling_labels:
            # augment
            generator.augment_label_maps(settings.generator_labelling_atlases,
                                         settings.generator_aug_labelling_features,
                                         settings.generator_aug_labelling_labels,
                                         settings.generator_aug_single_labelling_features,
                                         settings.generator_aug_single_labelling_labels)

        if settings.generator_house_expo:
            generator.convert_house_expo()
        else:
            generator.label_maps(settings.generator_labelling_atlases,
                                 settings.generator_labelling_features,
                                 settings.generator_labelling_labels,
                                 settings.generator_single_labelling_features,
                                 settings.generator_single_labelling_labels)
