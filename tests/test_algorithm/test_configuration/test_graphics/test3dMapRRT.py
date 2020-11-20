import time
import pyautogui
import os
from constants import DATA_PATH


pyautogui.click(1003, 218)

# Pick 3d cube with RRT
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Map.png'), confidence=0.5)
pyautogui.click(x+160, y+5)
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH,'3dCube.png'), confidence=0.9)
pyautogui.click(x, y)
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH,'Algorithm.png'), confidence=0.5)
pyautogui.click(x+150, y)
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'rrt2.png'), confidence=0.9)
pyautogui.click(x, y)
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
time.sleep(5)

# rotate map to pick a pos
pyautogui.press('a', presses=5500)
time.sleep(0.5)
pyautogui.rightClick(997, 532)

# make traversibles transparent for the RRT
x, y= pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH,'traversibles.png'), confidence=0.5)
pyautogui.click(x-120, y)

# #run algo
pyautogui.press('t')
time.sleep(1)

# take ss
pyautogui.press('o')
time.sleep(1)
pyautogui.press('p')

