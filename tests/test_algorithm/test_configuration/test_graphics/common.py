import Xlib.display
import pyautogui
import numpy as np

import atexit
import subprocess
import sys
import os
import time
import argparse
from typing import List, Optional, Callable, Tuple

SRC_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))), "src")
DATA_PATH = os.path.join(os.path.dirname(SRC_PATH), os.path.join("data"))
TEST_DATA_PATH = os.path.join(DATA_PATH, "test")
RESOURCES_PATH = os.path.join(SRC_PATH, "resources")

g_procs: List[Tuple[subprocess.Popen, Callable[[subprocess.Popen], None]]] = []

def try_delete_file(fn) -> None:
    """
    Attempt to delete a file.

    :param fn: file to delete
    """

    try:
        os.remove(fn)
    except OSError as e:
        print(e, file=sys.stderr)

def kill_processes() -> None:
    global g_procs

    for p, on_kill in g_procs:
        if on_kill:
            on_kill(p)
        try:
            p.communicate(timeout=0.5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.communicate()

    g_procs = []

def launch_process(cmd, on_kill: Callable[[subprocess.Popen], None] = None) -> None:
    global g_procs

    print(" ".join(cmd))
    g_procs.append((subprocess.Popen(cmd), on_kill))

def setup(args) -> None:
    atexit.register(kill_processes)
    sys.path.append(SRC_PATH)

    if args.spawn_display:
        launch_process(["Xvfb", args.spawn_display, "-screen", "0", "2112x1376x24", "-fbdir", "/var/tmp"])
        os.environ['DISPLAY'] = args.spawn_display

    if args.view_display:
        display = os.environ['DISPLAY'] if args.view_display == "auto" else args.view_display
        launch_process(["x11vnc", "-display", display, "-localhost"])
        launch_process(["vncviewer", "-display", ":0"])

    if not args.no_restore_resources_at_exit:
        atexit.register(restore_resources)

    if not args.no_rm_config_file:
        try_delete_file("./.pathbench.json")

    if "DISPLAY" in os.environ:
        pyautogui._pyautogui_x11._display = Xlib.display.Display(os.environ['DISPLAY'])

    if not args.no_launch_visualiser:
        launch_process([sys.executable, os.path.join(SRC_PATH, 'main.py'), '-v', '-Vwindowed-fullscreen'],
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

    parser.add_argument('test_args', metavar='TEST_ARG', nargs='*', help="arguments for test framework")

    if no_restore_resources_at_exit:
        sys.argv.append("--no-restore-resources-at-exit")

    if no_launch_visualiser:
        sys.argv.append("--no-launch-visualiser")

    if no_rm_config_file:
        sys.argv.append("--no-rm-config-file")

    args = parser.parse_args()
    print("args:{}".format(args))

    # remove all custom arguments for testing framework
    for a in ("--spawn-display", "--view-display", "--no-restore-resources-at-exit", "--no-launch-visualiser", "--no-rm-config-file"):
        for i in range(1, len(sys.argv)):
            s = sys.argv[i]
            if s.startswith(a):
                sys.argv.pop(i)
                if len(s) == len(a) and len(sys.argv) > i and not sys.argv[i].startswith("-"):
                    sys.argv.pop(i)  # pop value
                break

    setup(args)

def restore_resources() -> None:
    cmd = ["git", "restore", "--source=HEAD", "--staged", "--worktree", "--", RESOURCES_PATH]
    subprocess.check_call(cmd)

    cmd = ["git", "clean", "-dxf", "--", RESOURCES_PATH]
    subprocess.check_call(cmd)

def wait_for(rel_img: str, delay: float = 0.5, max_attempts: int = 30, confidence: float = 0.5) -> None:
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
