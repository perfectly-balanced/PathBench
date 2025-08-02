import structures
import torch
import dill
import algorithms.configuration.maps.dense_map
from typing import NamedTuple

class Size(NamedTuple):
    width: int
    height: int

# 修复元类定义，适配Python 3.8+
class Point(NamedTuple):
    x: int
    y: int


DenseMap = algorithms.configuration.maps.dense_map.DenseMap

convert_classes = {
    "Size": lambda s: structures.Size(s.width, s.height),
    "Point": lambda p: structures.Point(p.x, p.y),
    "DenseMap": lambda d: algorithms.configuration.maps.dense_map.DenseMap(d.grid)
}
names = ("structures.Point", "structures.Size", "algorithms.configuration.maps.dense_map.DenseMap")

def recursive_replace_loaded_objects(obj, depth=3):
    cls_name = obj.__class__.__qualname__
    if cls_name in convert_classes:
        print(f"Replacing object {cls_name}")
        return convert_classes[cls_name](obj)

    for attribute in dir(obj):
        if attribute.startswith("__") and attribute.endswith("__"):
            continue
        attr = getattr(obj, attribute)
        if attr.__class__.__module__ == "builtins":
            continue
        cls_name = attr.__class__.__qualname__
        if cls_name in convert_classes:
            print("Replacing object attribute")
            print(f"Before: {getattr(obj, attribute)}")
            setattr(obj, attribute, convert_classes[cls_name](getattr(obj, attribute)))
            print(f"After: {getattr(obj, attribute)}")
        elif depth > 0:
            print(f"Recursing into attr {attribute}")
            try:
                setattr(obj, attribute, recursive_replace_loaded_objects(getattr(obj, attribute), depth - 1))
            except AttributeError:
                pass
    return obj

def load(fname):
    old_classes = tuple([eval(i) for i in names])
    exec(", ".join(i for i in names) + " = " + ", ".join(i.split(".")[-1] for i in names))

    loaded_obj = dill.load(open(fname, "rb"), ignore=True)

    exec(", ".join(i for i in names) + " = old_classes")

    return recursive_replace_loaded_objects(loaded_obj)
