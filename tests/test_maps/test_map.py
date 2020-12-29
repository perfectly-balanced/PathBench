import unittest

import copy
from typing import List

from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from structures import Point, Size


class TestMap(unittest.TestCase):
    def test_reset_none(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map1.reset()
        self.assertEqual([], map1.trace)
        self.assertEqual(map1.at(Point(0, 1)), map1.AGENT_ID)

    def test_reset_normal(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])
        map1.move_agent(Point(1, 1))
        map1.reset()
        self.assertEqual([], map1.trace)
        self.assertEqual(map1.at(Point(0, 1)), map1.AGENT_ID)

    def test_neighbours_bounds(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID, DenseMap.WALL_ID],
            [DenseMap.AGENT_ID, DenseMap.CLEAR_ID, DenseMap.WALL_ID, DenseMap.EXTENDED_WALL_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])

        ns = map1.get_next_positions(Point(0, 1))
        self.assertEqual({Point(1, 1), Point(1, 2), Point(0, 2)}, set(ns))

    def test_neighbours_all(self) -> None:
        map1: DenseMap = DenseMap([
            [DenseMap.WALL_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.AGENT_ID, DenseMap.CLEAR_ID],
            [DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.CLEAR_ID, DenseMap.GOAL_ID],
        ])

        ns: List[Point] = map1.get_next_positions(Point(1, 1))
        self.assertEqual(
            {Point(x=1, y=0), Point(x=2, y=0), Point(x=2, y=1), Point(x=2, y=2), Point(x=1, y=2), Point(x=0, y=2),
             Point(x=0, y=1)}, set(ns))

    def test_copy(self) -> None:
        map1: Map = Map(Size(2, 3))
        with self.assertRaises(Exception):
            copy.copy(map1)

    def test_deep_copy(self) -> None:
        map1: Map = Map(Size(2, 3))
        with self.assertRaises(Exception):
            copy.deepcopy(map1)

    def test_ne(self) -> None:
        self.assertNotEqual(Map(Size(3, 2)), 5)
