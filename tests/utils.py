import subprocess
import os
import sys

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
            p.kill()
            p.communicate()

    g_procs = []

def launch_process(cmd, on_kill: Callable[[subprocess.Popen], None] = None) -> None:
    global g_procs

    print(" ".join(cmd))
    g_procs.append((subprocess.Popen(cmd), on_kill))

def handle_display_args(args) -> None:
    global g_old_display

    if args.spawn_display:
        def on_display_kill(*discard) -> None:
            global g_old_display

            if g_old_display is None:
                del os.environ['DISPLAY']
            else:
                os.environ['DISPLAY'] = g_old_display

        launch_process(["Xvfb", args.spawn_display, "-screen", "0", "2112x1376x24", "-fbdir", "/var/tmp"], on_display_kill)

        g_old_display = os.environ['DISPLAY'] if 'DISPLAY' in os.environ else None
        os.environ['DISPLAY'] = args.spawn_display

    if args.view_display:
        display = os.environ['DISPLAY'] if args.view_display == "auto" else args.view_display
        launch_process(["x11vnc", "-display", display, "-localhost"])
        launch_process(["vncviewer", "-display", ":0"])

def make_src_modules_importable() -> None:
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
