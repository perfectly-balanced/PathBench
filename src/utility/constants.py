import os

from utility.compatibility import Final

ROOT_PATH: Final = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC_PATH: Final = os.path.join(ROOT_PATH, "src")
DATA_PATH: Final = os.path.join(ROOT_PATH, "data")
GUI_DATA_PATH: Final = os.path.join(DATA_PATH, "gui")
TEST_DATA_PATH: Final = os.path.join(DATA_PATH, "test")
