"""

import unittest

import copy

from algorithms.basic_testing import BasicTesting
from algorithms.classic.a_star import AStar
from algorithms.classic.testing.wavefront_testing import WavefrontTesting
from algorithms.configuration.configuration import Configuration
from algorithms.classic.wavefront import Wavefront
from maps import Maps
from simulator.services.debug import DebugLevel


class TestConfiguration(unittest.TestCase):
    def test_str(self):
        config: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                              WavefrontTesting,
                                              True, 0, 0, DebugLevel.MEDIUM)
        self.assertEqual("\\\\\\\\\\""Configuration: {
	write_debug_level: DebugLevel.MEDIUM, 
	grid_display: False, 
	graphics: True, 
	key_frame_speed: 0, 
	key_frame_speed: 0, 
	window_size: Size(width=500, height=500), 
	algorithms: <class 'algorithms.classic.wavefront.Wavefront'>, 
	map_type: <class 'algorithms.configuration.maps.dense_map.DenseMap'>, 
	map: DenseMap: {
		size: Size(width=500, height=500), 
		agent: Agent: {position: Point(x=40, y=40), radius: 10}, 
		goal: Goal: {position: Point(x=460, y=460), radius: 10}, 
		obstacles: 11289
	}
}"\\\\\\\\\\"", str(config))

    def test_copy(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = copy.copy(config1)
        self.assertEqual(config1, config2)

    def test_deep_copy(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = copy.deepcopy(config1)
        self.assertEqual(config1, config2)

    def test_eq_without_grid_display(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertEqual(config1, config2)

    def test_eq_with_grid_display(self) -> None:
        config1: Configuration = Configuration(True, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(True, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertEqual(config1, config2)

    def test_ne_grid_display(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(True, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_map(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.grid_map_labyrinth, Wavefront, WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_testing_type(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               BasicTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_grid_algorithm_type(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), AStar,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_key_frame_speed(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 5, 0, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_key_frame_skip(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 5, DebugLevel.MEDIUM)
        self.assertNotEqual(config1, config2)

    def test_ne_write_debug_level(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.HIGH)
        self.assertNotEqual(config1, config2)

    def test_ne_instance(self) -> None:
        config1: Configuration = Configuration(False, Maps.pixel_map_one_obstacle.convert_to_dense_map(), Wavefront,
                                               WavefrontTesting,
                                               True, 0, 0, DebugLevel.MEDIUM)
        config2: int = 3
        self.assertNotEqual(config1, config2)


if __name__ == '__main__':
    TestConfiguration().run()

"""
