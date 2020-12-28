import time
import os
import sys

if __name__ == "__main__":
    from common import init, destroy
else:
    from .common import init, destroy

init()

from constants import RESOURCES_PATH, TEST_DATA_PATH  # noqa: E402
import pyautogui  # noqa: E402

x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth_new.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm_new.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.5)
pyautogui.click(x, y)

# change animations to slow
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'animations.png'), confidence=0.9)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'slow.png'), confidence=0.9)
pyautogui.click(x, y)

# update
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
pyautogui.press('w', presses=5500)

# start and end goals
pyautogui.click(900, 815)
time.sleep(0.5)
pyautogui.rightClick(1311, 875)
time.sleep(0.5)

# pick colors and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
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
time.sleep(1)

# save into state 1
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'one.png'), confidence=0.4)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'save.png'), confidence=0.5)
pyautogui.click(x, y)

# go to state 2
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'two.png'), confidence=0.7)
pyautogui.click(x, y)

# change colors
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
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

# restore
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'restore.png'), confidence=0.7)
pyautogui.click(x, y)

# check if all states are working
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'three.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'four.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'five.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'six.png'), confidence=0.9)
pyautogui.click(x, y)

destroy()
