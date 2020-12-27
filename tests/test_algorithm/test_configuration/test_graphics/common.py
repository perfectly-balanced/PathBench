import Xlib.display
import pyautogui
import numpy as np

import atexit
import subprocess
import sys
import os
import time
from typing import List

SRC_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))), "src")
DATA_PATH = os.path.join(os.path.dirname(SRC_PATH), os.path.join("data"))
RESOURCES_PATH = os.path.join(SRC_PATH, "resources")

g_proc = None

def try_delete_file(fn) -> None:
    """
    Attempt to delete a file.

    :param fn: file to delete
    """

    try:
        os.remove(fn)
    except OSError as e:
        print(e, file=sys.stderr)

def init(update_sys_path: bool = True, virtual_display_support: bool = True) -> None:
    if virtual_display_support:
        pyautogui._pyautogui_x11._display = Xlib.display.Display(os.environ['DISPLAY'])

    if update_sys_path:
        sys.path.append(SRC_PATH)

def launch_visualiser(args: List[str] = ['-v'], rm_config_file: bool = True, full_screen: bool = True) -> None:
    global g_proc

    if rm_config_file:
        try_delete_file(os.path.join(SRC_PATH, ".pathbench.json"))

    cmd = []
    if full_screen:
        cmd.extend(["env", "PATH_BENCH_FULLSCREEN=1"])
    cmd.extend([sys.executable, os.path.join(SRC_PATH, 'main.py')])
    cmd.extend(args)

    print(" ".join(cmd))
    g_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    atexit.register(destroy_visualiser)

    pyautogui.moveTo(1, 1)
    wait_for('on_launch.png')

    return g_proc

def destroy_visualiser() -> None:
    global g_proc
    if g_proc is None:
        return

    pyautogui.press('esc')

    try:
        output = g_proc.communicate(timeout=0.5)[0].decode("utf-8")
    except subprocess.TimeoutExpired:
        g_proc.kill()
        output = g_proc.communicate()[0].decode("utf-8")

    if g_proc.returncode != 0:
        print("Visualiser exited uncleanly (%d)%s" % (g_proc.returncode, ":\n" + output if output else ""))
        sys.exit(1)

    g_proc = None
    atexit.unregister(destroy_visualiser)

# Lower values better
def mse(img_a, img_b) -> float:
    err = np.sum((img_a.astype("float") - img_b.astype("float")) ** 2)
    err /= float(img_a.shape[0] * img_a.shape[1])
    return err

def wait_for(rel_img: str, delay: float = 0.5, max_attempts: int = 30, confidence: float = 0.5) -> None:
    img = os.path.join(DATA_PATH, rel_img)
    for _ in range(max_attempts):
        time.sleep(delay)
        try:
            if pyautogui.locateCenterOnScreen(img, confidence=confidence) is not None:
                return
        except pyautogui.PyAutoGUIException:
            continue
    raise RuntimeError("Unable to locate '{}' on screen".format(img))
