import Xlib.display
import numpy as np

import atexit
import subprocess
import sys
import os
import time
import argparse
from typing import List, Optional, Callable, Tuple

# add 'PathBench/tests' to system path for module imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from utils import make_src_modules_importable, handle_display_args, launch_process, kill_processes, try_delete_file  # noqa: E402

# add src folder to system path
make_src_modules_importable()

from constants import SRC_PATH, RESOURCES_PATH, TEST_DATA_PATH

g_restore_resources: bool = False

def setup(args) -> None:
    global g_restore_resources
    
    atexit.register(destroy)

    handle_display_args(args)

    g_restore_resources = not args.no_restore_resources_at_exit

    if not args.no_rm_config_file:
        try_delete_file("./.pathbench.json")

    # import pyautogui after setting up virtual display otherwise import
    # will fail on system with no display as $DISPLAY hasn't been set.
    import pyautogui

    # enforce correct display being used
    pyautogui._pyautogui_x11._display = Xlib.display.Display(os.environ['DISPLAY'])

    if not args.no_launch_visualiser:
        launch_process([sys.executable, os.path.join(SRC_PATH, 'main.py'), '-v', '-Vwindowed-fullscreen', '-Vaudio-library-name=null'],
                       on_kill=lambda _: pyautogui.press('esc'))

        pyautogui.moveTo(1, 1)
        wait_for('update.png')

def init(no_restore_resources_at_exit: bool = False, no_launch_visualiser: bool = False, no_rm_config_file: bool = False) -> None:
    parser = argparse.ArgumentParser(prog="common.py",
                                     description="PathBench individual test initialiser",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--spawn-display", nargs='?', default=None, const=":99", help="spawn a virtual display for testing graphics and sets $DISPLAY")
    parser.add_argument("--view-display", nargs='?', default=None, const="auto", help="open an interactive view of a virtual display (defaults to view $DISPLAY)")
    parser.add_argument("--no-restore-resources-at-exit", action="store_true", help="do not restore and clean resources folder when test exits")
    parser.add_argument("--no-launch-visualiser", action="store_true", help="do not launch PathBench visualiser")
    parser.add_argument("--no-rm-config-file", action="store_true", help="do not delete PathBench configuration file")

    if no_restore_resources_at_exit:
        sys.argv.append("--no-restore-resources-at-exit")

    if no_launch_visualiser:
        sys.argv.append("--no-launch-visualiser")

    if no_rm_config_file:
        sys.argv.append("--no-rm-config-file")

    args = parser.parse_known_args()[0]
    print("args:{}".format(args))

    setup(args)

def destroy() -> None:
    global g_restore_resources

    kill_processes()

    if g_restore_resources:
        restore_resources()
    
    atexit.unregister(destroy)

def restore_resources() -> None:
    cmd = ["git", "restore", "--source=HEAD", "--staged", "--worktree", "--", RESOURCES_PATH]
    subprocess.check_call(cmd)

    cmd = ["git", "clean", "-dxf", "--", RESOURCES_PATH]
    subprocess.check_call(cmd)

def wait_for(rel_img: str, delay: float = 0.5, max_attempts: int = 30, confidence: float = 0.5) -> None:
    import pyautogui

    img = os.path.join(TEST_DATA_PATH, rel_img)
    for _ in range(max_attempts):
        time.sleep(delay)
        try:
            if pyautogui.locateCenterOnScreen(img, confidence=confidence) is not None:
                return
        except pyautogui.PyAutoGUIException:
            continue
    raise RuntimeError("Unable to locate '{}' on screen".format(img))

# Lower values better
def mse(img_a, img_b) -> float:
    err = np.sum((img_a.astype("float") - img_b.astype("float")) ** 2)
    err /= float(img_a.shape[0] * img_a.shape[1])
    return err
