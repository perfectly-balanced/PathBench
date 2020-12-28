import time
import pyautogui
import os
import sys

if __name__ == "__main__":
    from common import init, destroy, TEST_DATA_PATH
else:
    from .common import init, destroy, TEST_DATA_PATH

init()


# Select map Labyrinth, RRT algorithm
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
pyautogui.click(x+160, y+5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth_new.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
pyautogui.click(x+150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'rrt2.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
pyautogui.press('w', presses=5500)

# pick colours and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x-78, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.9)
pyautogui.click(x, y)


# start and end goals
pyautogui.rightClick(900, 964)
time.sleep(0.5)
pyautogui.click(1511, 815)
time.sleep(1)

pyautogui.press('t')
time.sleep(1)

# take texture ss
pyautogui.press('o')

# make traversibles transparent for the other ss
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x-120, y)

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
pyautogui.click(x-120, y)

destroy()
