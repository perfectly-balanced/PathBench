from structures import Colour

def blend_colours(src: Colour, dst: Colour):
    wda = dst.a * (1 - src.a)  # weighted dst alpha
    a = src.a + wda
    d = (a if a != 0 else 1)
    r = (src.r * src.a + dst.r * wda) / d
    g = (src.g * src.a + dst.g * wda) / d
    b = (src.b * src.a + dst.b * wda) / d
    return Colour(r, g, b, a)
