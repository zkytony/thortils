# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

__all__ = ['getch',
           'getkey',
           'SPECIAL_KEYS']

######### Keys
# https://code.activestate.com/recipes/134892/
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()

# Read key; deals with multi-character keys
UP    = "up"
DOWN  = "down"
RIGHT = "right"
LEFT  = "left"


SPECIAL_KEYS = {
    '\x1b[A': UP,
    '\x1b[B': DOWN,
    '\x1b[C': RIGHT,
    '\x1b[D': LEFT
}

def _read_key():
    ch = getch()
    if ch == "\x1b":
        ch = getch()
        if ch == "[":
            ch = getch()
            key = SPECIAL_KEYS["\x1b[" + ch]
            return key
    return ch

getkey = _read_key
