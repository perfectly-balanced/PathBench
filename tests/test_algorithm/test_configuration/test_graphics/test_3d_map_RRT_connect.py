import time
import pyautogui
import os
import sys

sys.path.append(os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))),
    "src"))
from constants import DATA_PATH

pyautogui.click(1003, 218)

# Pick 3d cube with RRT-connect
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, '3dCube.png'), confidence=0.9)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Algorithm.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'rrt_connect.png'), confidence=0.8)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
time.sleep(5)

# rotate map to pick a pos
pyautogui.press('a', presses=5500)
time.sleep(0.5)
pyautogui.rightClick(997, 532)

# make traversables transparent for the RRT
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# #run algo
pyautogui.press('t')
time.sleep(1)

# take ss
pyautogui.press('o')
time.sleep(1)

# change trace color
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'trace.png'), confidence=0.7)
pyautogui.click(x - 70, y)
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'color2.png'), confidence=0.8)
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
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)
