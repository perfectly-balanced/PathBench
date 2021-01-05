import time
import os
import unittest

if __name__ == "__main__":
    from common import init, destroy, take_screenshot, wait_for, compare_images
else:
    from .common import init, destroy, take_screenshot, wait_for, compare_images


def graphics_test() -> None:
    from utility.constants import DATA_PATH, TEST_DATA_PATH
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
    wait_for('initialised.png')

    # go into state 1
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'one.png'), confidence=0.7)
    pyautogui.click(x, y)

    # pick colors and other modifications of the map
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 83, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'orange.png'), confidence=0.9)
    pyautogui.click(x, y)
    time.sleep(0.5)

    # run
    pyautogui.press('t')
    wait_for('done.png')
    time.sleep(0.2)

    # change trace color
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace.png'), confidence=0.8)
    pyautogui.click(x - 70, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_1.png'), confidence=0.8)
    pyautogui.click(x, y)
    time.sleep(1.5)

    transparent_1 = take_screenshot()
    time.sleep(1)

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'save.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(2)

    # go to state 2
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'two.png'), confidence=0.7)
    pyautogui.click(x, y)

    transparent_2 = take_screenshot()

    # change colors
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 85, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lime.png'), confidence=0.9)
    pyautogui.click(x, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace.png'), confidence=0.8)
    pyautogui.click(x - 70, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.7)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'visited.png'), confidence=0.7)
    pyautogui.click(x - 70, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_2.png'), confidence=0.7)
    pyautogui.click(x, y)

    # test restore changes
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'restore.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(1.5)

    transparent_restored_2 = take_screenshot()

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'one.png'), confidence=0.7)
    pyautogui.click(x, y)
    time.sleep(2.5)

    transparent_restored_1 = take_screenshot()

    # check if all states are working
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'three.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'four.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'five.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'six.png'), confidence=0.9)
    pyautogui.click(x, y)

    compare_images(transparent_restored_1, transparent_1, threshold=1)
    compare_images(transparent_restored_2, transparent_2, threshold=1)


class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
