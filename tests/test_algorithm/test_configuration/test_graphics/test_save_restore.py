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
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
    pyautogui.click(x + 160, y + 5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth_new.png'), confidence=0.5)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
    pyautogui.click(x + 150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.5)
    pyautogui.click(x, y)

    # start and end goals coordinate input
    pyautogui.click(342, 545)
    pyautogui.write('17')
    pyautogui.doubleClick(422, 545)
    pyautogui.write('9')
    pyautogui.doubleClick(342, 617)
    pyautogui.write('4')
    pyautogui.doubleClick(422, 617)
    pyautogui.write('5')

    # update
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
    pyautogui.click(x, y)
    pyautogui.press('w', presses=5000)

    # go into state 1
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'one.png'), confidence=0.7)
    pyautogui.click(x, y)

    # pick colors and other modifications of the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 78, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'orange.png'), confidence=0.9)
    pyautogui.click(x, y)
    time.sleep(0.5)

    # run
    pyautogui.press('t')
    time.sleep(0.5)
    # change trace color
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace.png'), confidence=0.7)
    pyautogui.click(x - 70, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_1.png'), confidence=0.8)
    pyautogui.click(x, y)
    time.sleep(2)

    # take texture ss
    pyautogui.press('o')
    time.sleep(2)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'save.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(2)
    # go to state 2
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'two.png'), confidence=0.7)
    pyautogui.click(x, y)

    # change colors
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 78, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lime.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace.png'), confidence=0.7)
    pyautogui.click(x - 70, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.7)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'visited.png'), confidence=0.7)
    pyautogui.click(x - 70, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_2.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(2)
    # get the latest taken screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_1 = max(list_of_ss, key=os.path.getctime)

    # take texture ss
    pyautogui.press('o')
    time.sleep(3)

    # get the latest taken screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_2 = max(list_of_ss, key=os.path.getctime)

    # test restore changes
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'restore.png'), confidence=0.7)
    pyautogui.click(x, y)

    # take texture ss
    pyautogui.press('o')
    time.sleep(3)

    # get the latest taken screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_not_restored_2 = max(list_of_ss, key=os.path.getctime)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'one.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(0.5)

    # take texture ss
    pyautogui.press('o')
    time.sleep(3)

    # get the latest taken screenshot
    list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
    transparent_restored_1 = max(list_of_ss, key=os.path.getctime)

    # check if all states are working
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'three.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'four.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'five.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'six.png'), confidence=0.9)
    pyautogui.click(x, y)

    # compare the 2 new screenshots with the expected ones
    transparent_1 = cv.imread(transparent_1)
    transparent_2 = cv.imread(transparent_2)
    transparent_not_restored_2 = cv.imread(transparent_not_restored_2)
    transparent_restored_1 = cv.imread(transparent_restored_1)

    print(mse(transparent_restored_1, transparent_1))
    print(mse(transparent_not_restored_2, transparent_2))

    # Small error allowed for the top screen high res ss, usually very close to 0
    THRESHOLD = 1
    mse_1 = mse(transparent_restored_1, transparent_1)
    mse_2 = mse(transparent_not_restored_2, transparent_2)
    assert mse_1 < THRESHOLD, mse_1
    assert mse_2 > THRESHOLD, mse_2

class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
