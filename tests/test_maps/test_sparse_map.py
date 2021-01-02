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


class TestSparseMap(unittest.TestCase):
    def test_copy(self) -> None:
        map1: SparseMap = Maps.pixel_map_one_obstacle
        map2: SparseMap = copy.copy(map1)
        self.assertEqual(map1, map2)

    def test_deep_copy(self) -> None:
        map1: SparseMap = Maps.pixel_map_one_obstacle
        map2: SparseMap = copy.deepcopy(map1)
        self.assertEqual(map1, map2)

    def test_eq(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        self.assertEqual(map1, map2)

    def test_ne_size(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(400, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_agent(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 15),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_goal(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(100, 160), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_obstacle(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(90, 100), 40)],
                                    Goal(Point(180, 160), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_all(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: SparseMap = SparseMap(Size(100, 200),
                                    Agent(Point(10, 20), 10),
                                    [Obstacle(Point(100, 100), 35)],
                                    Goal(Point(180, 10), 10))
        self.assertNotEqual(map1, map2)

    def test_ne_dense(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: DenseMap = SparseMap(Size(200, 200),
                                   Agent(Point(20, 20), 10),
                                   [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                   Goal(Point(180, 160), 10)).convert_to_dense_map()
        self.assertNotEqual(map1, map2)

    def test_ne_instance(self) -> None:
        map1: SparseMap = SparseMap(Size(200, 200),
                                    Agent(Point(20, 20), 10),
                                    [Obstacle(Point(40, 40), 10), Obstacle(Point(100, 100), 40)],
                                    Goal(Point(180, 160), 10))
        map2: int = 2
        self.assertNotEqual(map1, map2)

    def test_eq_dense_map(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: SparseMap = SparseMap(
            Size(4, 3),
            Agent(Point(0, 1)),
            [Obstacle(Point(0, 0)), Obstacle(Point(1, 0)), Obstacle(Point(2, 0)), Obstacle(Point(3, 0))],
            Goal(Point(3, 2))
        )
        self.assertEqual(map2, map1)

    def test_convert_to_dense_map(self) -> None:
        map1: SparseMap = SparseMap(
            Size(4, 3),
            Agent(Point(0, 1)),
            [Obstacle(Point(0, 0)), Obstacle(Point(1, 0)), Obstacle(Point(2, 0)), Obstacle(Point(3, 0))],
            Goal(Point(3, 2))
        )
        map2: DenseMap = map1.convert_to_dense_map()
        self.assertEqual(map1, map2)

    def test_move_agent_normal(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        map1.move_agent(Point(1, 1))
        self.assertEqual(Point(1, 1), map1.agent.position)
        self.assertTrue([Trace(Point(1, 1))], map1.trace)

    def test_move_agent_no_trace(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        map1.move_agent(Point(1, 1), True)
        self.assertEqual(Point(1, 1), map1.agent.position)
        self.assertEqual([], map1.trace)

    def test_move_agent_out_of_bounds(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        map1.move_agent(Point(-1, 0))
        self.assertEqual(Point(0, 1), map1.agent.position)
        self.assertEqual([Trace(Point(0, 1))], map1.trace)

    def test_is_goal_reached_normal(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertTrue(map1.is_goal_reached(Point(3, 2)))

    def test_is_goal_reached_false(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_goal_reached(Point(2, 2)))

    def test_is_goal_reached_out_of_bounds(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_goal_reached(Point(-1, -1)))

    def test_is_valid_position_normal(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.EXTENDED_WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertTrue(map1.is_agent_valid_pos(Point(1, 1)))
        self.assertTrue(map1.is_agent_valid_pos(Point(0, 1)))
        self.assertTrue(map1.is_agent_valid_pos(Point(3, 2)))
        self.assertTrue(map1.is_agent_valid_pos(Point(0, 0)))

    def test_is_valid_position_invalid(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.EXTENDED_WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertFalse(map1.is_agent_valid_pos(Point(1, 0)))
        self.assertFalse(map1.is_agent_valid_pos(Point(-1, -1)))

    def test_str(self) -> None:
        map1: SparseMap = DenseMap([
            [DenseMap.EXTENDED_WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ]).convert_to_sparse_map()
        self.assertEqual("""SparseMap: {
		size: Size(4, 3), 
		agent: Agent: {position: Point(0, 1), radius: 0}, 
		obstacles: {
			size: 4, 
			entities: [
				Obstacle: {position: Point(1, 0), radius: 0}, 
				Obstacle: {position: Point(2, 0), radius: 0}, 
				Obstacle: {position: Point(3, 0), radius: 0}, 
				Obstacle: {position: Point(3, 1), radius: 0}, 
			]
		}, 
		goal: Goal: {position: Point(3, 2), radius: 0}
	}""", str(map1))

    def test_str_debug_level_3(self) -> None:
        services: Services = Mock()
        services.settings.simulator_write_debug_level = DebugLevel.HIGH

        map1: SparseMap = DenseMap([
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [2, 3, 0, 0, 0, 0, 0, 0, 0, 0]
        ], services=services).convert_to_sparse_map()

        self.assertEqual("""SparseMap: {
		size: Size(10, 3), 
		agent: Agent: {position: Point(0, 2), radius: 0}, 
		obstacles: {
			size: 20, 
			entities: [
				Obstacle: {position: Point(0, 0), radius: 0}, 
				Obstacle: {position: Point(1, 0), radius: 0}, 
				Obstacle: {position: Point(2, 0), radius: 0}, 
				Obstacle: {position: Point(3, 0), radius: 0}, 
				Obstacle: {position: Point(4, 0), radius: 0}, 
				Obstacle: {position: Point(5, 0), radius: 0}, 
				Obstacle: {position: Point(6, 0), radius: 0}, 
				Obstacle: {position: Point(7, 0), radius: 0}, 
				Obstacle: {position: Point(8, 0), radius: 0}, 
				Obstacle: {position: Point(9, 0), radius: 0}, 
				Obstacle: {position: Point(0, 1), radius: 0}, 
				Obstacle: {position: Point(1, 1), radius: 0}, 
				Obstacle: {position: Point(2, 1), radius: 0}, 
				Obstacle: {position: Point(3, 1), radius: 0}, 
				Obstacle: {position: Point(4, 1), radius: 0}, 
				Obstacle: {position: Point(5, 1), radius: 0}, 
				Obstacle: {position: Point(6, 1), radius: 0}, 
				Obstacle: {position: Point(7, 1), radius: 0}, 
				Obstacle: {position: Point(8, 1), radius: 0}, 
				Obstacle: {position: Point(9, 1), radius: 0}, 
			]
		}, 
		goal: Goal: {position: Point(1, 2), radius: 0}
	}""", str(map1))
