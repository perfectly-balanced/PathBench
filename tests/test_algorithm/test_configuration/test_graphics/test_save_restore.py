import time
import pyautogui
import os
import sys

sys.path.append(os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))),
    "src"))
from constants import DATA_PATH

pyautogui.click(1003, 218)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Labyrinth.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Algorithm.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'A.png'), confidence=0.5)
pyautogui.click(x, y)

# change animations to slow
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'animations.png'), confidence=0.9)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'slow.png'), confidence=0.9)
pyautogui.click(x, y)

# update
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
pyautogui.press('w', presses=5500)

# start and end goals
pyautogui.click(900, 815)
time.sleep(0.5)
pyautogui.rightClick(1311, 875)
time.sleep(0.5)

# pick colors and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 78, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'orange.png'), confidence=0.9)
pyautogui.click(x, y)
time.sleep(0.5)


# run
pyautogui.press('t')
time.sleep(0.5)
# change trace color
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'trace.png'), confidence=0.7)
pyautogui.click(x - 70, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'color1.png'), confidence=0.8)
pyautogui.click(x, y)
time.sleep(1)

# save into state 1
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'one.png'), confidence=0.4)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'save.png'), confidence=0.5)
pyautogui.click(x, y)

# go to state 2
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'two.png'), confidence=0.7)
pyautogui.click(x, y)

# change colors
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 78, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'lime.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'trace.png'), confidence=0.7)
pyautogui.click(x - 70, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'lightblue.png'), confidence=0.7)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'visited.png'), confidence=0.7)
pyautogui.click(x - 70, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'color2.png'), confidence=0.7)
pyautogui.click(x, y)

# restore
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'restore.png'), confidence=0.7)
pyautogui.click(x, y)

# check if all states are working
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'three.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'four.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'five.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'six.png'), confidence=0.9)
pyautogui.click(x, y)







