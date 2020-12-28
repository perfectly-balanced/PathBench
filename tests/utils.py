import subprocess
import os
import sys
import time
from functools import partial

from typing import List, Tuple, Callable

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

    for p, on_kill in reversed(g_procs):
        if on_kill:
            on_kill(p)
        try:
            p.communicate(timeout=0.5)
        except subprocess.TimeoutExpired:
            p.send_signal(subprocess.signal.SIGTERM)
            try:
                p.communicate(timeout=1)
            except subprocess.TimeoutExpired:
                p.kill()
                p.communicate()

    g_procs = []

def launch_process(cmd, on_kill: Callable[[subprocess.Popen], None] = None) -> None:
    global g_procs

    print(" ".join(cmd))
    g_procs.append((subprocess.Popen(cmd), on_kill))

def handle_display_args(args) -> None:
    if args.spawn_display:
        cmd = ["Xvfb", args.spawn_display, "-screen", "0", "2112x1376x24", "-fbdir", "/var/tmp"]
        if 'DISPLAY' in os.environ:
            def on_display_kill(display, *discard) -> None:
                os.environ['DISPLAY'] = display
            launch_process(cmd, partial(on_display_kill, os.environ['DISPLAY']))
        else:
            def on_display_kill(*discard) -> None:
                del os.environ['DISPLAY']
            launch_process(cmd, on_display_kill)
        
        os.environ['DISPLAY'] = args.spawn_display
        time.sleep(0.8) # wait for display to actually launch

    if args.view_display:
        display = os.environ['DISPLAY'] if args.view_display == "auto" else args.view_display
        launch_process(["x11vnc", "-display", display, "-localhost"])
        launch_process(["vncviewer", "-display", ":0"])

def make_src_modules_importable() -> None:
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
