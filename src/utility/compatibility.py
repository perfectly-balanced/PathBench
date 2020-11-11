try:
    from typing import Final
except ImportError: # work-around for Python < 3.8
    from typing import Tuple
    Final = Tuple