import time
import os
import sys
import glob
import cv2 as cv
import pyautogui

if __name__ == "__main__":
    from common import init, mse, RESOURCES_PATH, TEST_DATA_PATH
else:
    from .common import init, mse, RESOURCES_PATH, TEST_DATA_PATH

init()


# Select map Labyrinth, A* algorithm, update
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.5)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm.png'), confidence=0.5)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.5)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.5)
pyautogui.click(x, y)
pyautogui.press('w', presses=5500)

# pick colours and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 78, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.9)
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
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# hide gui
pyautogui.press('c')
time.sleep(1)
pyautogui.press('v')
time.sleep(1)

# get the latest taken screenshot (transparent one)
list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
transparent = max(list_of_ss, key=os.path.getctime)
print("transparent file is: " + transparent)

# take the full screen ss
pyautogui.press('p')
time.sleep(0.5)

# get latest screenshot
list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
full_screen = max(list_of_ss, key=os.path.getctime)

print("full screen file is: " + full_screen)

# revert changes
pyautogui.press('c')
time.sleep(0.5)
pyautogui.press('v')
time.sleep(0.5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.5)
pyautogui.click(x - 120, y)

# compare the 2 new screenshots with the expected ones
expected_transparent = cv.imread(os.path.join(TEST_DATA_PATH, "expectedLATransparent.png"))
expected_full = cv.imread(os.path.join(TEST_DATA_PATH, "expectedLAFull.png"))
transparent = cv.imread(transparent)
full_screen = cv.imread(full_screen)

# Small error allowed for the top screen high res ss, usually very close to 0
print(mse(expected_transparent, transparent))
assert mse(expected_transparent, transparent) < 10

# Bigger error allowed for full screen ss
print(mse(expected_full, full_screen))
assert mse(expected_full, full_screen) < 1600
print(mse(expected_full, full_screen))
