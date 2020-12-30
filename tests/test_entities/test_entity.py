import unittest

import copy

from algorithms.configuration.entities.entity import Entity
from structures import Point


class TestEntity(unittest.TestCase):
    def test_copy(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = copy.copy(entity1)
        self.assertEqual(entity1, entity2)

    def test_deep_copy(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = copy.deepcopy(entity1)
        self.assertEqual(entity1, entity2)

    def test_str(self) -> None:
        entity: Entity = Entity(Point(2, 3), 10)
        self.assertEqual("Entity: {position: Point(2, 3), radius: 10}", str(entity))

    def test_eq(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = Entity(Point(2, 3), 10)
        self.assertEqual(entity1, entity2)

    def test_ne_pos(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = Entity(Point(2, 5), 10)
        self.assertNotEqual(entity1, entity2)

    def test_ne_radius(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = Entity(Point(2, 3), 15)
        self.assertNotEqual(entity1, entity2)

    def test_ne_all(self) -> None:
        entity1: Entity = Entity(Point(2, 3), 10)
        entity2: Entity = Entity(Point(2, 15), 15)
        self.assertNotEqual(entity1, entity2)

    def test_ne_instance(self) -> None:
        entity1: int = 2
        entity2: Entity = Entity(Point(2, 3), 10)
        self.assertNotEqual(entity1, entity2)
