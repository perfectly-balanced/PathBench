import structures, torch, dill
from typing import NamedTuple

class Size(NamedTuple):
    width: int
    height: int

class Point(NamedTuple, torch.Tensor):
    x: int
    y: int

convert_classes = {
    "Size": lambda s: structures.Size(s.width, s.height),
    "Point": lambda p: structures.Point(p.x, p.y)
}

def load(fp):

    old_classes = (structures.Point, structures.Size)
    structures.Point, structures.Size = Point, Size
    
    loaded_obj = dill.load(fp, ignore=True)

    structures.Point, structures.Size = old_classes

    
    for attribute in dir(loaded_obj):
        cls_name = getattr(loaded_obj, attribute).__class__.__qualname__
        print(attribute, cls_name)
        if cls_name in convert_classes:
            print("Replacing loaded_object")
            print(f"Before: {getattr(loaded_obj, attribute)}")
            setattr(loaded_obj, attribute, convert_classes[cls_name](getattr(loaded_obj, attribute)))
            print(f"After: {getattr(loaded_obj, attribute)}")
    return loaded_obj