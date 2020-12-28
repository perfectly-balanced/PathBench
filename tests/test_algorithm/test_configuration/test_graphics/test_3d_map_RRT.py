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

    # Pick 3d cube with RRT
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
    pyautogui.click(x + 160, y + 5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, '3d_cube.png'), confidence=0.9)
    pyautogui.click(x, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.55)
    pyautogui.click(x + 150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'rrt2.png'), confidence=0.9)
    pyautogui.click(x, y)

    # start and end goals coordinate input
    pyautogui.click(342, 545)
    pyautogui.write('1')
    pyautogui.doubleClick(422, 545)
    pyautogui.write('1')
    pyautogui.doubleClick(502, 545)
    pyautogui.write('1')
    time.sleep(0.5)
    pyautogui.doubleClick(342, 617)
    pyautogui.write('13')
    time.sleep(0.5)
    pyautogui.doubleClick(422, 617)
    pyautogui.write('1')
    pyautogui.doubleClick(502, 617)
    pyautogui.write('13')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
    pyautogui.click(x, y)
    time.sleep(5)

    # make traversables transparent for the RRT
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 120, y)

    # #run algo
    pyautogui.press('t')
    time.sleep(4)

    # take ss
    pyautogui.press('o')
    time.sleep(3)

    # get latest screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_1 = max(list_of_ss, key=os.path.getctime)
    time.sleep(2)

    # compare the screenshot with the expected one
    expected_transparent_1 = cv.imread(os.path.join(TEST_DATA_PATH, "3d_rrt.png"))
    transparent_1 = cv.imread(transparent_1)

    print(mse(expected_transparent_1, transparent_1))

    # Some error allowed due to RRT running differently sometimes
    THRESHOLD = 1500
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
