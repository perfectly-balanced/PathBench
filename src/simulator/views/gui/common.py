try:
    from typing import Final
except ImportError:
    Final = 'Final[Colour]'
from structures import Colour

WINDOW_BG_COLOUR: Final = Colour(0.5, 0.5, 0.5, 1.0)
WIDGET_BG_COLOUR: Final = Colour(0.3, 0.3, 0.3, 1.0)
