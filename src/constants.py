import os

from utility.compatibility import Final

DATA_PATH: Final = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
