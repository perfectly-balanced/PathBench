import sys
import os
import pyautogui
import time

if __name__ == "__main__":
    from common import init, TEST_DATA_PATH
else:
    from .common import init, TEST_DATA_PATH

init()


# Pick 3d cube with RRT-connect
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, '3d_cube.png'), confidence=0.8)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'rrt_connect.png'), confidence=0.8)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
time.sleep(5)

# rotate map to pick a pos
pyautogui.press('a', presses=5500)
time.sleep(0.5)
pyautogui.rightClick(997, 532)

# make traversables transparent for the RRT
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# #run algo
pyautogui.press('t')
time.sleep(1)

# take ss
pyautogui.press('o')
time.sleep(1)

# change trace color
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'trace.png'), confidence=0.7)
pyautogui.click(x - 70, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'colour_2.png'), confidence=0.8)
pyautogui.click(x, y)
time.sleep(1)


# hide gui
pyautogui.press('c')
time.sleep(0.5)
pyautogui.press('v')
time.sleep(1)

# take scene ss
pyautogui.press('p')
time.sleep(1)

# restore changes
pyautogui.press('v')
time.sleep(0.5)
pyautogui.press('c')
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)
