import unittest
import copy

from structures import Size


class TestSize(unittest.TestCase):
    def test_copy(self) -> None:
        size1: Size = Size(2, 3)
        size2: Size = copy.copy(size1)
        self.assertEqual(size1, size2)

    def test_deep_copy(self) -> None:
        size1: Size = Size(2, 3)
        size2: Size = copy.deepcopy(size1)
        self.assertEqual(size1, size2)

    def test_str(self) -> None:
        size = Size(2, 3)
        self.assertEqual("Size(2, 3)", str(size))

    def test_str_3d(self) -> None:
        size = Size(2, 3, 4)
        self.assertEqual("Size(2, 3, 4)", str(size))

    def test_eq(self) -> None:
        size1: Size = Size(2, 3)
        size2: Size = Size(2, 3)
        self.assertEqual(size1, size2)

    def test_eq_3d(self) -> None:
        size1: Size = Size(2, 3, 4)
        size2: Size = Size(2, 3, 4)
        self.assertEqual(size1, size2)

    def test_ne_pos(self) -> None:
        size1: Size = Size(2, 3)
        size2: Size = Size(2, 5)
        size3: Size = Size(1, 3)
        self.assertNotEqual(size1, size2)
        self.assertNotEqual(size1, size3)

    def test_ne_pos_3d(self) -> None:
        size1: Size = Size(2, 3, 4)
        size2: Size = Size(2, 3, 5)
        self.assertNotEqual(size1, size2)

    def test_ne_dim(self) -> None:
        size1: Size = Size(2, 3)
        size2: Size = Size(2, 3, 0)
        self.assertNotEqual(size1, size2)

    def test_ne_instance(self) -> None:
        size1: int = 2
        size2: Size = Size(2, 3)
        self.assertNotEqual(size1, size2)

    def test_conversion(self) -> None:
        size: Size = Size(2, 3, 4)
        self.assertEqual(tuple(size), (2, 3, 4))
        self.assertEqual(size.values, (2, 3, 4))
        self.assertEqual(list(size), [2, 3, 4])
