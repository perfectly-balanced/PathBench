import unittest

from structures.tracked_set import TrackedSet


class TestTrackedSet(unittest.TestCase):
    # Creates new instance for each test
    def setUp(self):
        self.tr_set = TrackedSet()
        self.tr_set.add(1)
        self.tr_set.add(2)
        self.tr_set.add(3)
        self.tr_set.add(4)
        self.tr_set.add(5)

    def test_len(self) -> None:
        self.assertEqual(self.tr_set.__len__(), 5)

    def test_added_duplicates(self) -> None:
        self.tr_set.add(4)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.assertEqual(len(self.tr_set.modified), 5)

    def test_added(self) -> None:
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.assertEqual(self.tr_set.modified[1], 2)
        self.tr_set.add(200)
        self.assertEqual(self.tr_set.modified[-1], 200)
        self.assertEqual(len(self.tr_set.modified), 6)

    def test_clear(self) -> None:
        self.assertEqual(self.tr_set.__len__(), 5)
        self.assertEqual(len(self.tr_set.modified), 5)
        self.tr_set.clear()
        self.assertEqual(self.tr_set.__len__(), 0)
        self.assertEqual(len(self.tr_set.modified), 10)

    def test_difference_update(self) -> None:
        self.assertEqual(self.tr_set.__len__(), 5)
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.difference_update(1, 2, 33, 34, 35, 36)
        self.assertEqual(self.tr_set.__len__(), 3)
        self.assertEqual(len(self.tr_set.modified), 7)
        self.assertEqual(self.tr_set.modified[-1], 2)

    def test_intersection_update(self) -> None:
        self.assertEqual(self.tr_set.__len__(), 5)
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.intersection_update(1, 2, 33, 34, 35, 36)
        self.assertEqual(self.tr_set.__len__(), 2)
        self.assertEqual(len(self.tr_set.modified), 8)
        self.assertEqual(self.tr_set.modified[-1], 5)

    def test_pop(self) -> None:
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.pop()
        self.assertEqual(len(self.tr_set.modified), 6)
        self.assertEqual(self.tr_set.modified[-1], 1)

    def test_remove(self) -> None:
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.remove(4)
        self.assertEqual(len(self.tr_set.modified), 6)
        self.assertEqual(self.tr_set.modified[-1], 4)

    def test_discard(self) -> None:
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.discard(4)
        self.assertEqual(len(self.tr_set.modified), 6)
        self.assertEqual(self.tr_set.modified[-1], 4)
        self.tr_set.discard(0)
        self.assertEqual(len(self.tr_set.modified), 6)
        self.assertEqual(self.tr_set.modified[-1], 4)

    def test_update(self) -> None:
        self.assertEqual(self.tr_set.__len__(), 5)
        self.assertEqual(len(self.tr_set.modified), 5)
        self.assertEqual(self.tr_set.modified[-1], 5)
        self.tr_set.update(1, 2, 33, 34, 35, 36)
        self.assertEqual(self.tr_set.__len__(), 9)
        self.assertEqual(len(self.tr_set.modified), 9)
        self.assertEqual(self.tr_set.modified[-1], 36)

    # def test_symmetric_update(self) -> None:
    #     self.assertEqual(self.tr_set.__len__(), 5)
    #     self.assertEqual(len(self.tr_set.modified), 5)
    #     self.assertEqual(self.tr_set.modified[-1], 5)
    #     set_test = {1, 2, 33, 34, 35, 36}
    #     self.tr_set.symmetric_difference_update(set_test)
    #     print(self.tr_set._set)
    #     self.assertEqual(self.tr_set.__len__(), 8)
    #     self.assertEqual(len(self.tr_set.modified), 7)
    #     self.assertEqual(self.tr_set.modified[-1], 2)
    #
