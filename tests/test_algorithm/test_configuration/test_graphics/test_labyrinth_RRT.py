import time
import os
import glob
import cv2 as cv
import unittest

if __name__ == "__main__":
    from common import init, destroy, mse
else:
    from .common import init, destroy, mse

def graphics_test() -> None:
    from constants import RESOURCES_PATH, TEST_DATA_PATH
    import pyautogui

    # Select map Labyrinth, RRT algorithm
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
    pyautogui.click(x+160, y+5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth_new.png'), confidence=0.5)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
    pyautogui.click(x+150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'rrt2.png'), confidence=0.9)
    pyautogui.click(x, y)

    # start and end goals coordinate input
    pyautogui.click(342, 545)
    pyautogui.write('17')
    pyautogui.doubleClick(422, 545)
    pyautogui.write('15')
    pyautogui.doubleClick(342, 617)
    pyautogui.write('17')
    pyautogui.doubleClick(422, 617)
    pyautogui.write('4')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
    pyautogui.click(x, y)
    time.sleep(2)
    # pick colours and do other modifications on the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x-85, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lime.png'), confidence=0.9)
    pyautogui.click(x, y)

    pyautogui.press('t')
    time.sleep(2)

    # take texture ss
    pyautogui.press('o')

    # wait until ss is saved
    time.sleep(3)

    # get latest screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_1 = max(list_of_ss, key=os.path.getctime)

    # compare the new screenshot with the expected one
    expected_transparent_1 = cv.imread(os.path.join(TEST_DATA_PATH, "labyrinth_RRT_1.png"))
    transparent_1 = cv.imread(transparent_1)

    THRESHOLD = 3700
    mse_1 = mse(expected_transparent_1, transparent_1)
    assert mse_1 < THRESHOLD, mse_1


class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
