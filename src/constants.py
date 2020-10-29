import os
try:
    from typing import Final
except ImportError:
    Final = 'Final[Colour]'

DATA_PATH: Final = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
