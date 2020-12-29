import unittest
import numpy as np

from structures.tracked_grid import TrackedGrid


class TestTrackedGrid(unittest.TestCase):
    # Creates new instance for each test
    def setUp(self):
        self.tr_grid = TrackedGrid(np.full((5, 6, 7), 0, dtype=np.int32))

        self.tr_grid[0, 0, 1] = 1
        self.tr_grid[0, 2, 0] = 2
        self.tr_grid[3, 0, 0] = 3

        self.tr_grid[0][0][2] = 2
        self.tr_grid[0][3][0] = 3
        self.tr_grid[4][0][0] = 4
        self.tr_grid[4][0][0] = 8

    def test_shape(self) -> None:
        self.assertEqual(self.tr_grid.shape, (5, 6, 7))
        self.assertEqual(self.tr_grid[0].shape, (6, 7))
        self.assertEqual(self.tr_grid[0][0].shape, (7,))
        self.assertEqual(self.tr_grid[0, 0].shape, (7,))

    def test_dtype(self) -> None:
        self.assertEqual(self.tr_grid.dtype, np.int32)
        self.assertEqual(self.tr_grid[0].dtype, np.int32)
        self.assertEqual(self.tr_grid[0][0].dtype, np.int32)
        self.assertTrue(type(self.tr_grid[0][0][0]) is np.int32)
        self.assertEqual(self.tr_grid[0, 0].dtype, np.int32)
        self.assertTrue(type(self.tr_grid[0, 0, 0]) is np.int32)
        self.assertTrue(type(self.tr_grid[0][0, 0]) is np.int32)
        self.assertTrue(type(self.tr_grid[0, 0][0]) is np.int32)

    def test_ndim(self) -> None:
        self.assertEqual(self.tr_grid.ndim, 3)
        self.assertEqual(self.tr_grid[0].ndim, 2)
        self.assertEqual(self.tr_grid[0][0].ndim, 1)
        self.assertEqual(self.tr_grid[0, 0].ndim, 1)

    def test_values(self) -> None:
        expected = np.full((5, 6, 7), 0, dtype=np.int32)
        expected[0, 0, 1] = 1
        expected[0, 2, 0] = 2
        expected[3, 0, 0] = 3

        expected[0][0][2] = 2
        expected[0][3][0] = 3
        expected[4][0][0] = 8

        for idx in np.ndindex(self.tr_grid.shape):
            i, j, k = idx
            self.assertEqual(self.tr_grid[idx], expected[idx])
            self.assertEqual(self.tr_grid[i][j][k], expected[i][j][k])
            self.assertEqual(self.tr_grid[i, j][k], expected[i, j][k])
            self.assertEqual(self.tr_grid[i][j, k], expected[i][j, k])

    def test_tracking(self) -> None:
        expected = [((0, 0, 1), 0), ((0, 2, 0), 0), ((3, 0, 0), 0), ((0, 0, 2), 0), ((0, 3, 0), 0), ((4, 0, 0), 0), ((4, 0, 0), 4)]
        self.assertEqual(self.tr_grid.modified, expected)
        self.tr_grid.clear_tracking_data()
        self.assertEqual(self.tr_grid.modified, [])
