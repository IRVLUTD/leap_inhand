"""Interactive keyboard control for the 16-motor hand (Linux).

Sets current limit, return delay 0, and position mode. Torque starts off and
is disabled on Q or Ctrl-C.

Run from leap_ethernet/ in a real terminal:

    uv run python scripts/keyboard_control_linux.py

Arguments:
    none    No command-line flags. Keys while running:
            T    toggle torque
            I    move to open position
            C    move to closed position
            R    read present positions
            H    reprint this help
            Q    quit (torque off)
            Non-letter keys are ignored.
"""

import select
import sys
import termios
import tty

from keyboard_control import run


def read_key() -> str | None:
    key = sys.stdin.read(1)
    if key == "\x1b":
        while select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.read(1)
        return None
    if len(key) != 1 or not key.isalpha():
        return None
    return key.lower()


def main() -> None:
    stdin_fd = sys.stdin.fileno()
    original_terminal = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        run(read_key)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_terminal)


if __name__ == "__main__":
    main()
