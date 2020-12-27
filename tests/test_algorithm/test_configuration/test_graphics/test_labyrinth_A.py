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
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'map.png'), confidence=0.6)
pyautogui.click(x + 160, y + 5)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'labyrinth.png'), confidence=0.3)
pyautogui.click(x, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'algorithm.png'), confidence=0.4)
pyautogui.click(x + 150, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'a_star.png'), confidence=0.6)
pyautogui.click(x, y)


# start and end goals coordinate input
pyautogui.click(342, 545)
pyautogui.write('17')
pyautogui.doubleClick(422, 545)
pyautogui.write('9')
pyautogui.doubleClick(342, 617)
pyautogui.write('6')
pyautogui.doubleClick(422, 617)
pyautogui.write('4')

x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'update.png'), confidence=0.6)
pyautogui.click(x, y)
# pyautogui.press('w', presses=5000)

time.sleep(2)
# pick colours and other modifications of the map
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.6)
pyautogui.click(x - 78, y)
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'lightblue.png'), confidence=0.8)
pyautogui.click(x, y)

pyautogui.press('t')
time.sleep(0.5)

# take texture ss
pyautogui.press('o')

# make traversables transparent
x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.6)
pyautogui.click(x - 120, y)

# get the latest taken screenshot
list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
transparent_1 = max(list_of_ss, key=os.path.getctime)

# take the second texture ss
pyautogui.press('o')

# wait until ss is saved
time.sleep(3)

# get latest screenshot
list_of_ss = glob.glob(os.path.join(RESOURCES_PATH, 'screenshots/*.png'))
transparent_2 = max(list_of_ss, key=os.path.getctime)

x, y = pyautogui.locateCenterOnScreen(os.path.join(TEST_DATA_PATH, 'traversables.png'), confidence=0.6)
pyautogui.click(x - 120, y)

# compare the 2 new screenshots with the expected ones
expected_transparent_1 = cv.imread(os.path.join(TEST_DATA_PATH, "labyrinth_A_1.png"))
expected_transparent_2 = cv.imread(os.path.join(TEST_DATA_PATH, "labyrinth_A_2.png"))
transparent_1 = cv.imread(transparent_1)
transparent_2 = cv.imread(transparent_2)

print(mse(expected_transparent_1, transparent_1))
print(mse(expected_transparent_2, transparent_2))

# Small error allowed for the top screen high res ss, usually very close to 0
assert mse(expected_transparent_1, transparent_1) < 1
(mse(expected_transparent_2, transparent_2))
assert mse(expected_transparent_2, transparent_2) < 1
