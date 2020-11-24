try:
    from typing import Final
except ImportError:  # work-around for Python < 3.8
    from typing import Tuple
    Final = Tuple

try:
    import ompl
    HAS_OMPL: Final[bool] = True
except:
    HAS_OMPL: Final[bool] = False
