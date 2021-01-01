import time
import os
import glob
import cv2 as cv
import unittest

if __name__ == "__main__":
    from common import init, destroy, mse, wait_for, screenshot_saved
else:
    from .common import init, destroy, mse, wait_for, screenshot_saved


def graphics_test() -> None:
    from utility.constants import DATA_PATH, TEST_DATA_PATH
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

    wait_for('initialised.png')

    # run algo
    pyautogui.press('t')

    wait_for('traversables_new.png')

    # pick colours and other modifications of the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 85, y)
    time.sleep(0.5)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_1.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(0.5)

    wait_for('traversables_new.png')
    wait_for('done.png')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 125, y)

    # take ss
    file_num = len(glob.glob(os.path.join(DATA_PATH, 'screenshots/*.png')))
    pyautogui.press('o')

    # get latest screenshot
    screenshot_saved(file_num)
    list_of_ss = glob.glob(os.path.join(DATA_PATH, 'screenshots/*.png'))
    transparent_1 = max(list_of_ss, key=os.path.getctime)

    wait_for('traversables_new.png')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 125, y)

    # take new transparent ss
    pyautogui.press('o')

    # wait until ss is saved
    screenshot_saved(file_num + 1)

    # get latest screenshot
    list_of_ss = glob.glob(os.path.join(DATA_PATH, 'screenshots/*.png'))
    transparent_2 = max(list_of_ss, key=os.path.getctime)

    # compare the 2 new screenshots with the expected ones
    time.sleep(1)
    expected_transparent_1 = cv.imread(os.path.join(TEST_DATA_PATH, "3d_A_1.png"))
    expected_transparent_2 = cv.imread(os.path.join(TEST_DATA_PATH, "3d_A_2.png"))
    transparent_1 = cv.imread(transparent_1)
    transparent_2 = cv.imread(transparent_2)

    # Small error allowed for the top screen high res ss, usually very close to 0
    THRESHOLD = 30
    mse_1 = mse(expected_transparent_1, transparent_1)
    mse_2 = mse(expected_transparent_2, transparent_2)
    assert mse_1 < THRESHOLD, mse_1
    assert mse_2 < THRESHOLD, mse_2


class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
