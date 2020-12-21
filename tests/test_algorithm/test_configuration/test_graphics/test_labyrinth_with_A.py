from common import init, mse
init()

from constants import DATA_PATH

import time
import os
import sys
import glob
import cv2
import pyautogui

pyautogui.click(1003, 218)

# Select map Labyrinth, A* algorithm, update
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Labyrinth.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'Algorithm.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'A.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
pyautogui.press('w', presses=5500)

# pick colours and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 78, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'lightblue.png'), confidence=0.9)
pyautogui.click(x, y)

# start and end goals
pyautogui.rightClick(900, 960)
time.sleep(0.5)
pyautogui.click(1511, 815)
time.sleep(1)

pyautogui.press('t')
time.sleep(0.5)

# take texture ss
pyautogui.press('o')

# make traversables transparent for the full scene ss
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# hide gui
pyautogui.press('c')
time.sleep(1)
pyautogui.press('v')
time.sleep(1)

# get the latest taken screenshot (transparent one)
list_of_ss = glob.glob('../../../../src/resources/screenshots/*.png')
transparent = max(list_of_ss, key=os.path.getctime)
print("transparent file is: " + transparent)

# take the full screen ss
pyautogui.press('p')
time.sleep(0.5)

# get latest screenshot
list_of_ss = glob.glob('../../../../src/resources/screenshots/*.png')
full_screen = max(list_of_ss, key=os.path.getctime)

print("full screen file is: " + full_screen)

# revert changes
pyautogui.press('c')
time.sleep(0.5)
pyautogui.press('v')
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# compare the 2 new screenshots with the expected ones
expected_transparent = cv2.imread("../../../../src/resources/screenshots/expectedLATransparent.png")
expected_full = cv2.imread("../../../../src/resources/screenshots/expectedLAFull.png")
transparent = cv2.imread(transparent)
full_screen = cv2.imread(full_screen)

# Small error allowed for the top screen high res ss, usually very close to 0
print(mse(expected_transparent, transparent))
assert mse(expected_transparent, transparent) < 10

# Bigger error allowed for full screen ss
print(mse(expected_full, full_screen))
assert mse(expected_full, full_screen) < 1600
print(mse(expected_full, full_screen))
