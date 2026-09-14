"""Interactive keyboard control for the 16-motor hand (Windows).

Sets current limit, return delay 0, and position mode. Torque starts off and
is disabled on Q or Ctrl-C.

Run from leap_ethernet/:

    uv run python scripts/keyboard_control_windows.py

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

import msvcrt

from keyboard_control import run


def read_key() -> str | None:
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        msvcrt.getwch()
        return None
    if len(key) != 1 or not key.isalpha():
        return None
    return key.lower()


if __name__ == "__main__":
    run(read_key)
