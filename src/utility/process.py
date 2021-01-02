import subprocess
import os
import sys
import time
from functools import partial
from typing import List, Tuple, Callable

g_procs: List[Tuple[subprocess.Popen, Callable[[subprocess.Popen], None]]] = []

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
