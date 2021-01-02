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


class TestDenseMap(unittest.TestCase):
    def test_copy(self) -> None:
        map1: DenseMap = Maps.grid_map_labyrinth
        map2: DenseMap = copy.copy(map1)
        self.assertEqual(map1, map2)

    def test_deep_copy(self) -> None:
        map1: DenseMap = Maps.grid_map_labyrinth
        map2: DenseMap = copy.deepcopy(map1)
        self.assertEqual(map1, map2)

    def test_eq(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertEqual(map1, map2)

    def test_ne_size(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_clear(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_agent(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_goal(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.GOAL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.EXTENDED_WALL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_wall(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_all(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.CLEAR_ID, DenseMap.EXTENDED_WALL_ID],
        ])
        self.assertNotEqual(map1, map2)

    def test_ne_sparse(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: DenseMap = DenseMap([
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.CLEAR_ID, DenseMap.EXTENDED_WALL_ID],
        ]).convert_to_sparse_map()
        self.assertNotEqual(map1, map2)

    def test_ne_instance(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map2: int = 3
        self.assertNotEqual(map1, map2)

    def test_eq_sparse_map(self) -> None:
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
        self.assertEqual(map1, map2)

    def test_convert_to_sparse_map(self) -> None:
        map1: DenseMap = Maps.grid_map_labyrinth
        map2: SparseMap = map1.convert_to_sparse_map()
        self.assertEqual(map1, map2)

    def test_move_agent_normal(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map1.move_agent(Point(1, 1))
        self.assertEqual(Point(1, 1), map1.agent.position)
        self.assertEqual(DenseMap.AGENT_ID, map1.at(Point(1, 1)))
        self.assertTrue([Trace(Point(1, 1))], map1.trace)

    def test_move_agent_no_trace(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map1.move_agent(Point(1, 1), True)
        self.assertEqual(Point(1, 1), map1.agent.position)
        self.assertEqual(DenseMap.AGENT_ID, map1.at(Point(1, 1)))
        self.assertEqual([], map1.trace)

    def test_move_agent_out_of_bounds(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map1.move_agent(Point(-1, 0))
        self.assertEqual(DenseMap.AGENT_ID, map1.at(Point(0, 1)))
        self.assertEqual([Trace(Point(0, 1))], map1.trace)

    def test_is_goal_reached_normal(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertTrue(map1.is_goal_reached(Point(3, 2)))

    def test_is_goal_reached_false(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertFalse(map1.is_goal_reached(Point(2, 2)))

    def test_is_goal_reached_out_of_bounds(self) -> None:
        map1 = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertFalse(map1.is_goal_reached(Point(-1, -1)))

    def test_is_valid_position_normal(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertTrue(map1.is_agent_valid_pos(Point(1, 1)))
        self.assertTrue(map1.is_agent_valid_pos(Point(0, 1)))
        self.assertTrue(map1.is_agent_valid_pos(Point(3, 2)))

    def test_is_valid_position_invalid(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.EXTENDED_WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertFalse(map1.is_agent_valid_pos(Point(1, 0)))
        self.assertFalse(map1.is_agent_valid_pos(Point(0, 0)))
        self.assertFalse(map1.is_agent_valid_pos(Point(-1, -1)))

    def test_str(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.EXTENDED_WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        self.assertEqual("""DenseMap: {
		size: Size(4, 3), 
		agent: Agent: {position: Point(0, 1), radius: 0}, 
		goal: Goal: {position: Point(3, 2), radius: 0}, 
		obstacles: 5, 
		grid: [
			4, 1, 1, 1, 
			2, 0, 0, 1, 
			0, 0, 0, 3, 
		]
	}""", str(map1))

    def test_str_debug_level_3(self) -> None:
        services: Services = Mock()
        services.settings.simulator_write_debug_level = DebugLevel.HIGH
        map1: SparseMap = SparseMap(
            Size(30, 30),
            Agent(Point(1, 2), 1),
            [Obstacle(Point(5, 5), 100)],
            Goal(Point(4, 3), 1),
            services
        ).convert_to_dense_map()

        self.assertEqual("""DenseMap: {
		size: Size(30, 30), 
		agent: Agent: {position: Point(1, 2), radius: 1}, 
		goal: Goal: {position: Point(4, 3), radius: 1}, 
		obstacles: 898, 
		grid: [
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
		]
	}""", str(map1))
