from structures import Colour
import numpy as np

def blend_colours(src: Colour, dst: Colour):
    wda = dst.a * (1 - src.a)  # weighted dst alpha
    cs = np.multiply(src.values, src.a) + np.multiply(dst.values, wda)

    a = src.a + wda
    if a != 0:
        cs = np.divide(cs, a)

    return Colour(*cs)
