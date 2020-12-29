import unittest

import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.trace import Trace
from structures import Point


class TestTrace(unittest.TestCase):
    def test_copy(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Trace = copy.copy(entity1)
        self.assertEqual(entity1, entity2)

    def test_deep_copy(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Trace = copy.deepcopy(entity1)
        self.assertEqual(entity1, entity2)

    def test_str(self) -> None:
        entity: Trace = Trace(Point(2, 3))
        self.assertEqual("Trace: {position: Point(2, 3)}", str(entity))

    def test_eq(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Trace = Trace(Point(2, 3))
        self.assertEqual(entity1, entity2)

    def test_ne_pos(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Trace = Trace(Point(2, 5))
        self.assertNotEqual(entity1, entity2)

    def test_ne_all(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Trace = Trace(Point(1, 15))
        self.assertNotEqual(entity1, entity2)

    def test_ne_instance(self) -> None:
        entity1: Trace = Trace(Point(2, 3))
        entity2: Entity = Entity(Point(2, 3), 1)
        self.assertNotEqual(entity1, entity2)
