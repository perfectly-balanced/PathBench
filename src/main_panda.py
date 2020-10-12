from simulator.views.panda.main_view import MainView
import random

if __name__ == "__main__":
    # test data
    map3d = {}
    for x in range(0, 25):
        map3d[x] = {}
        for y in range(0, 25):
            map3d[x][y] = {}
            for z in range(0, 25):
                map3d[x][y][z] = bool(random.getrandbits(1))

    view = MainView(map3d)
    view.run()