import unittest

from structures.tracked_heap import TrackedHeap


class TestTrackedHeap(unittest.TestCase):
    # Creates new instance for each test
    def setUp(self):
        self.tr_heap = TrackedHeap()
        self.tr_heap.push(1)
        self.tr_heap.push(2)
        self.tr_heap.push(3)
        self.tr_heap.push(4)
        self.tr_heap.push(5)

    def test_len(self) -> None:
        self.assertEqual(self.tr_heap.__len__(), 5)

    def test_added_duplicates(self) -> None:
        self.assertEqual(len(self.tr_heap.modified), 5)
        self.assertEqual(self.tr_heap.modified[-1], 5)
        self.tr_heap.push(4)
        self.assertEqual(self.tr_heap.modified[-1], 4)
        self.assertEqual(len(self.tr_heap.modified), 6)

    def test_added(self) -> None:
        self.assertEqual(len(self.tr_heap.modified), 5)
        self.tr_heap.push(200)
        self.assertEqual(self.tr_heap.modified[-1], 200)
        self.assertEqual(len(self.tr_heap.modified), 6)

    def test_removed(self) -> None:
        self.assertEqual(len(self.tr_heap.modified), 5)
        self.tr_heap.pop()
        self.assertEqual(self.tr_heap.modified[-1], 1)
        self.assertEqual(len(self.tr_heap.modified), 6)
