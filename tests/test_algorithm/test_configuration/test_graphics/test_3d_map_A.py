import time
import pyautogui
import os
import sys

if __name__ == "__main__":
    from common import init, TEST_DATA_PATH
else:
    from .common import init, TEST_DATA_PATH

init()


# Pick 3d cube with A*
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, '3d_cube.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
time.sleep(5)

# rotate map to pick a pos
pyautogui.press('a', presses=5500)
pyautogui.rightClick(997, 532)

# run algo
pyautogui.press('t')
time.sleep(1)

# pick colours and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 78, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_1.png'), confidence=0.7)
pyautogui.click(x, y)

# take transparent default ss
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)
pyautogui.press('o')
time.sleep(1)

# take scene ss
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)
pyautogui.press('c')
time.sleep(0.5)
pyautogui.press('v')
time.sleep(0.5)
pyautogui.press('p')
time.sleep(0.5)

# restore changes
pyautogui.press('v')
pyautogui.press('c')
