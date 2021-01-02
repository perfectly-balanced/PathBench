import numpy as np
from collections.abc import Iterable
from typing import Tuple

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

def flatten(l, ignored_values=[], depth: int = np.iinfo(np.int32).max):
    if depth == 0:
        for e in l:
            yield e
    else:
        for el in l:
            if isinstance(el, Iterable) and not isinstance(el, (str, bytes)):
                for el2 in flatten(el, ignored_values, depth-1):
                    if el2 not in ignored_values:
                        yield el2
            elif el not in ignored_values:
                yield el

def array_shape(a) -> Tuple[int, ...]:
    if isinstance(a[0], Iterable):
        return (len(a), *array_shape(a[0]))
    else:
        return (len(a),)

def static_class(cls):
    if getattr(cls, "_static_init_", None):
        cls._static_init_()
    return cls
