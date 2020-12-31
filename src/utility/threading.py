# Here we can choose from a selection of threading
# modules. That is, we have alternatives from the
# standard threading module that Panda3D recommends
# to use for safety, however, they are super-duper
# slow. Should only enable these when debugging
# Panda3D code.

_THREADING_TYPE = "STD"
assert _THREADING_TYPE in ("STD", "DIRECT", "DIRECT2")

if _THREADING_TYPE == "STD":
    from threading import *
elif _THREADING_TYPE == "DIRECT":
    from direct.stdpy.threading import *
elif _THREADING_TYPE == "DIRECT2":
    from direct.stdpy.threading2 import *

from typing import Optional, Callable  # noqa: E402
import time  # noqa: E402

def _cond_var_wait_for_direct(cv: Condition, predicate: Callable[[], bool], timeout: Optional[float] = None) -> bool:
    if timeout is None:
        while not predicate():
            cv.wait()
        return True
    else:
        start = time.time()
        while not predicate():
            if not cv.wait(timeout + start - time.time()):
                return False
        return True

def _cond_var_wait_for_std(cv: Condition, predicate: Callable[[], bool], timeout: Optional[float] = None) -> bool:
    return cv.wait_for(predicate, timeout)


"""
cond_var_wait_for()

-> Same behaviour as `cv.wait_for()`, but use to be compatible with all threading modules.
"""

if _THREADING_TYPE == "STD":
    cond_var_wait_for = _cond_var_wait_for_std
else:
    cond_var_wait_for = _cond_var_wait_for_direct
