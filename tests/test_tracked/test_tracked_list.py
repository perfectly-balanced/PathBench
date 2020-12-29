import unittest

from structures.tracked_list import TrackedList


class TestTrackedList(unittest.TestCase):

    # Creates new instance for each test
    def setUp(self):
        self.tr_list = TrackedList()
        self.tr_list.insert(0, 2)
        self.tr_list.insert(1, 3)
        self.tr_list.insert(2, 4)
        self.tr_list.insert(3, 12)
        self.tr_list.insert(4, 2)

    def test_len(self) -> None:
        self.assertEqual(self.tr_list.__len__(), 5)

    def test_added(self) -> None:
        self.assertEqual(self.tr_list.modified[-1], 2)
        self.assertEqual(self.tr_list.modified[1], 3)

    def test_removed(self) -> None:
        self.assertEqual(self.tr_list.modified[-1], 2)
        self.tr_list.__delitem__(2)
        self.assertEqual(self.tr_list.modified[-1], 4)

    def test_size(self) -> None:
        self.assertEqual(self.tr_list.__len__(), 5)
        self.assertEqual(self.tr_list.modified[-1], 2)
        self.tr_list.__delitem__(4)
        self.assertEqual(self.tr_list.__len__(), 4)
        self.assertEqual(self.tr_list.modified[-1], 2)

    def test_set_item(self) -> None:
        self.assertEqual(self.tr_list.__len__(), 5)
        self.assertEqual(len(self.tr_list.modified), 5)
        self.assertEqual(self.tr_list.modified[-1], 2)
        self.tr_list.__setitem__(2, 18)
        self.assertEqual(self.tr_list.modified[-1], 18)
        self.assertEqual(self.tr_list.__len__(), 5)
        self.assertEqual(len(self.tr_list.modified), 7)

    def test_insert(self) -> None:
        self.assertEqual(self.tr_list.modified[-1], 2)
        self.assertEqual(len(self.tr_list.modified), 5)
        self.tr_list.append(100)
        self.assertEqual(self.tr_list.modified[-1], 100)
        self.assertEqual(self.tr_list._list[-1], 100)
        self.assertEqual(len(self.tr_list.modified), 6)
