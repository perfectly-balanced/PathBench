import time
import os
import unittest

if __name__ == "__main__":
    from common import init, destroy, wait_for, take_screenshot
else:
    from .common import init, destroy, wait_for, take_screenshot


def graphics_test() -> None:
    from utility.constants import DATA_PATH, TEST_DATA_PATH
    import pyautogui

    # Select map Labyrinth, A* algorithm, update
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.6)
    pyautogui.click(x + 160, y + 5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth_new.png'), confidence=0.5)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.6)
    pyautogui.click(x + 150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.6)
    pyautogui.click(x, y)

    # start and end goals coordinate input
    pyautogui.click(342, 545)
    pyautogui.write('17')
    pyautogui.doubleClick(422, 545)
    pyautogui.write('9')
    pyautogui.doubleClick(342, 617)
    pyautogui.write('6')
    pyautogui.doubleClick(422, 617)
    pyautogui.write('4')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.6)
    pyautogui.click(x, y)

    wait_for('initialised.png')
    wait_for('traversables_new.png')

    # pick colours and other modifications of the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 85, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.8)
    pyautogui.click(x, y)

    pyautogui.press('t')
    wait_for('done.png')

    take_screenshot("labyrinth_A_1.png", threshold=350)

    wait_for('traversables_new.png')

    # make traversables transparent
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 125, y)

    take_screenshot("labyrinth_A_2.png", threshold=350)


class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
