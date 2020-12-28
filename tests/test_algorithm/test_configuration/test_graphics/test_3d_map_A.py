import time
import os
import glob
import cv2 as cv
import unittest

if __name__ == "__main__":
    from common import init, destroy, mse
else:
    from .common import init, destroy, mse

def graphics_unit_test() -> None:
    from constants import RESOURCES_PATH, TEST_DATA_PATH
    import pyautogui

    # Pick 3d cube with A*
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
    pyautogui.click(x + 160, y + 5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, '3d_cube.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
    pyautogui.click(x + 150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.5)
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

    # temporary
    time.sleep(5)

    # run algo
    pyautogui.press('t')

    time.sleep(1)

    # pick colours and other modifications of the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
    pyautogui.click(x - 78, y)

    time.sleep(0.5)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_1.png'), confidence=0.7)
    pyautogui.click(x, y)

    # take default ss
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
    pyautogui.click(x - 120, y)
    pyautogui.press('o')
    time.sleep(0.5)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
    pyautogui.click(x - 120, y)

    time.sleep(6)
    # get latest screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_1 = max(list_of_ss, key=os.path.getctime)

    # take new transparent ss
    pyautogui.press('o')
    time.sleep(0.5)


    # wait until ss is saved
    time.sleep(6)

    # get latest screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_2 = max(list_of_ss, key=os.path.getctime)

    # compare the 2 new screenshots with the expected ones
    expected_transparent_1 = cv.imread(os.path.join(TEST_DATA_PATH, "3d_A_1.png"))
    expected_transparent_2 = cv.imread(os.path.join(TEST_DATA_PATH, "3d_A_2.png"))
    transparent_1 = cv.imread(transparent_1)
    transparent_2 = cv.imread(transparent_2)

    # Small error allowed for the top screen high res ss, usually very close to 0
    assert mse(expected_transparent_1, transparent_1) < 11
    (mse(expected_transparent_2, transparent_2))
    assert mse(expected_transparent_2, transparent_2) < 11

class GraphicsTestCase(unittest.TestCase):
    def test(self):
        init()
        try:
            graphics_unit_test()
        finally:
            destroy()

if __name__ == "__main__":
    GraphicsTestCase().run()
