from panda3d.core import LVector3

def normalise(*args):
    v = LVector3(*args)
    v.normalize()
    return v
