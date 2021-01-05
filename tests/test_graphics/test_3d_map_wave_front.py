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

    # Pick Uniform Random Fill with Wave-Front
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
    pyautogui.click(x + 160, y + 5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'uniform.png'), confidence=0.9)
    pyautogui.click(x, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
    pyautogui.click(x + 150, y)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'wave_front.png'), confidence=0.7)
    pyautogui.click(x, y)

    # start and end goals coordinate input
    pyautogui.click(342, 545)
    pyautogui.write('6')
    pyautogui.doubleClick(422, 545)
    pyautogui.write('1')
    pyautogui.doubleClick(502, 545)
    pyautogui.write('2')
    time.sleep(0.5)
    pyautogui.doubleClick(342, 617)
    pyautogui.write('6')
    time.sleep(0.5)
    pyautogui.doubleClick(422, 617)
    pyautogui.write('5')
    pyautogui.doubleClick(502, 617)
    pyautogui.write('5')

    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
    pyautogui.click(x, y)

    wait_for('initialised.png')
    wait_for('traversables_new.png')

    # make traversables transparent for the RRT
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables_new.png'), confidence=0.5)
    pyautogui.click(x - 120, y)

    # #run algo
    pyautogui.press('t')
    wait_for('done.png')

    # change trace color
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace_new.png'), confidence=0.8)
    pyautogui.click(x - 70, y)
    time.sleep(0.5)
    x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_2.png'), confidence=0.8)
    pyautogui.click(x, y)

    take_screenshot("wave_front_3d.png")


class GraphicsTestCase(unittest.TestCase):
    def test(self):
        try:
            init()
            graphics_test()
        finally:
            destroy()


if __name__ == "__main__":
    GraphicsTestCase().test()
