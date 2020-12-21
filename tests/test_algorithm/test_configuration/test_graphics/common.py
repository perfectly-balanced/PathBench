import Xlib.display
import pyautogui
import sys
import os
import numpy as np

def init():
    pyautogui._pyautogui_x11._display = Xlib.display.Display(os.environ['DISPLAY'])
    sys.path.append(os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))),
        "src"))

# Lower values better
def mse(imageA, imageB):
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])
    return err
