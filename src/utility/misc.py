import numpy as np
from direct.stdpy.threading import Condition

from collections.abc import Iterable
from typing import Tuple, Callable, Optional
import time


def fmt_row(width, row):
    out = " | ".join(fmt_item(x, width) for x in row)
    return out


def fmt_item(x, l):
    if isinstance(x, np.ndarray):
        assert x.ndim == 0
        x = x.item()
    if isinstance(x, float):
        rep = "%g" % x
    else:
        rep = str(x)
    return " " * (l - len(rep)) + rep


def get_stats(loss, predictions, labels):
    cp = np.argmax(predictions.cpu().data.numpy(), 1)
    error = np.mean(cp != labels.cpu().data.numpy())
    return loss.item(), error


def print_stats(epoch, avg_loss, avg_error, num_batches, time_duration):
    print(
        fmt_row(10, [
            epoch + 1, avg_loss / num_batches, avg_error / num_batches,
            time_duration
        ]))


def print_header():
    print(fmt_row(10, ["Epoch", "Train Loss", "Train Error", "Epoch Time"]))


def exclude_from_dict(d, keys):
    return {key: d[key] for key in d if key not in keys}

def flatten(l, ignored_values=[]):
    for el in l:
        if isinstance(el, Iterable) and not isinstance(el, (str, bytes)):
            for el2 in flatten(el):
                if el2 not in ignored_values:
                    yield el2
        elif el not in ignored_values:
            yield el

def array_shape(a) -> Tuple[int, ...]:
    if isinstance(a[0], Iterable):
        return (len(a), *array_shape(a[0]))
    else:
        return (len(a),)

def cond_var_wait_for(cv: Condition, predicate: Callable[[], bool], timeout: Optional[float] = None) -> None:
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
