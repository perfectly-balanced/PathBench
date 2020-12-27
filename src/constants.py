import os

from utility.compatibility import Final

DATA_PATH: Final = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
GUI_DATA_PATH: Final = os.path.join(DATA_PATH, "gui")
TEST_DATA_PATH: Final = os.path.join(DATA_PATH, "test")
