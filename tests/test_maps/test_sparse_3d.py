import unittest

import copy
from unittest.mock import Mock

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.entities.trace import Trace
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.sparse_map import SparseMap
from maps.maps import Maps
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from structures import Size, Point


class TestSparseMap3D(unittest.TestCase):
    def test_copy(self) -> None:
        map1: SparseMap = Maps.pixel_map_one_obstacle_3d
        map2: SparseMap = copy.copy(map1)
        self.assertEqual(map1, map2)

    def test_deep_copy(self) -> None:
        map1: SparseMap = Maps.pixel_map_one_obstacle_3d
        map2: SparseMap = copy.deepcopy(map1)
        self.assertEqual(map1, map2)

    def test_eq(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        self.assertEqual(map1, map2)

    def test_ne_size(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(400, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_agent(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 10, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_goal(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 120, 120), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_obstacle(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(90, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_all(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: SparseMap = SparseMap(Size(100, 200, 200),
                                    Agent(Point(15, 20, 20), 10),
                                    [Obstacle(Point(40, 30, 40), 10), Obstacle(Point(100, 90, 100), 40)],
                                    Goal(Point(180, 150, 120), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_dense(self) -> None:
        map1: SparseMap = SparseMap(Size(20, 20, 20),
                                    Agent(Point(2, 2, 2), 1),
                                    [Obstacle(Point(4, 4, 4), 1), Obstacle(Point(10, 10, 10), 4)],
                                    Goal(Point(18, 16, 14), 1))
        map2: DenseMap = SparseMap(Size(20, 20, 20),
                                   Agent(Point(2, 2, 2), 1),
                                   [Obstacle(Point(4, 4, 4), 1), Obstacle(Point(10, 10, 10), 4)],
                                   Goal(Point(18, 16, 14), 1)).convert_to_dense_map()
        self.assertNotEqual(map1, map2)

    def test_ne_instance(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200, 200),
                                    Agent(Point(20, 20, 20), 10),
                                    [Obstacle(Point(40, 40, 40), 10), Obstacle(Point(100, 100, 100), 40)],
                                    Goal(Point(180, 160, 120), 10))
        map2: int = 2
        self.assertNotEqual(map1, map2)

    def test_eq_dense_map(self) -> None:
        map1: DenseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ])
        map2: SparseMap = SparseMap(
            Size(3, 2, 2),
            Agent(Point(0, 1, 0)),
            [Obstacle(Point(1, 0, 1)), Obstacle(Point(2, 0, 1))],
            Goal(Point(2, 1, 1))
        )
        self.assertEqual(map2, map1)

    def test_convert_to_dense_map(self) -> None:
        map1: SparseMap = SparseMap(
            Size(3, 2, 2),
            Agent(Point(0, 1, 0)),
            [Obstacle(Point(1, 0, 1)), Obstacle(Point(2, 0, 1))],
            Goal(Point(2, 1, 1))
        )
        map2: DenseMap = map1.convert_to_dense_map()
        self.assertEqual(map1, map2)

    def test_move_agent_normal(self) -> None:
        map1: DenseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        map1.move_agent(Point(0, 0, 0))
        self.assertEqual(Point(0, 0, 0), map1.agent.position)
        self.assertTrue([Trace(Point(0, 0, 0))], map1.trace)

    def test_move_agent_no_trace(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        map1.move_agent(Point(0, 0, 0), True)
        self.assertEqual(Point(0, 0, 0), map1.agent.position)
        self.assertEqual([], map1.trace)

    def test_move_agent_out_of_bounds(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        map1.move_agent(Point(-1, 0, 0))
        self.assertEqual(Point(0, 1, 0), map1.agent.position)
        self.assertEqual([Trace(Point(0, 1, 0))], map1.trace)

    def test_is_goal_reached_normal(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertTrue(map1.is_goal_reached(Point(2, 1, 1)))

    def test_is_goal_reached_false(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_goal_reached(Point(2, 1, 0)))

    def test_is_goal_reached_out_of_bounds(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_goal_reached(Point(-1, -1, -1)))

    def test_is_valid_position_normal(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertTrue(map1.is_agent_valid_pos(Point(0, 0, 0)))
        self.assertTrue(map1.is_agent_valid_pos(Point(1, 0, 0)))
        self.assertTrue(map1.is_agent_valid_pos(Point(2, 0, 0)))
        self.assertTrue(map1.is_agent_valid_pos(Point(0, 0, 1)))

    def test_is_valid_position_invalid(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_agent_valid_pos(Point(1, 0, 1)))
        self.assertFalse(map1.is_agent_valid_pos(Point(-1, -1, -1)))

    def test_str(self) -> None:
        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        self.assertEqual("""SparseMap: {
		size: Size(3, 2, 2), 
		agent: Agent: {position: Point(0, 1, 0), radius: 0}, 
		obstacles: {
			size: 2, 
			entities: [
				Obstacle: {position: Point(1, 0, 1), radius: 0}, 
				Obstacle: {position: Point(2, 0, 1), radius: 0}, 
			]
		}, 
		goal: Goal: {position: Point(2, 1, 1), radius: 0}
	}""", str(map1))

    def test_str_debug_level_3(self) -> None:
        service: Services = Mock()
        service.settings.simulator_write_debug_level = DebugLevel.HIGH

        map1: SparseMap = DenseMap([
            [[DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID], [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID]],
            [[DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID], [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID]]
        ]).convert_to_sparse_map()
        map1._services = service

        self.assertEqual("""SparseMap: {
		size: Size(3, 2, 2), 
		agent: Agent: {position: Point(0, 1, 0), radius: 0}, 
		obstacles: {
			size: 2, 
			entities: [
				Obstacle: {position: Point(1, 0, 1), radius: 0}, 
				Obstacle: {position: Point(2, 0, 1), radius: 0}, 
			]
		}, 
		goal: Goal: {position: Point(2, 1, 1), radius: 0}
	}""", str(map1))
