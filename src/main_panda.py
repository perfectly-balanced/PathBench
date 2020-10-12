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

    map3d[0][0][0] = True  # added for debugging purposes
    map3d[0][0][1] = True  # added for debugging purposes

    view = MainView(map3d, start_pos=(0, 0, 0), goal_pos=(0, 0, 1))

    # testing dynamic change of default colour
    view.map_mesh.default_colour = (0, 0, 0)
    view.map_mesh.default_colour = (0, 0, 1)
    view.map_mesh.default_colour = (1, 1, 1)

    view.run()
