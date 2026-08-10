"""Thin wrapper around the mms stdin/stdout API.

CRITICAL: mms reads this process's STDOUT as commands. Never print anything to
stdout except mms commands -- use log() (stderr) for everything else.

Response rules (from the mms README):
  * mazeWidth/Height, wall*, moveForward, turnLeft/Right, wasReset, ackReset
    each return one line -> we read stdin.
  * setWall/clearWall, setColor/clearColor/clearAllColor,
    setText/clearText/clearAllText return NOTHING -> we must NOT read stdin.
"""
import sys


class MouseCrashedError(Exception):
    pass


def log(*args):
    print(*args, file=sys.stderr)
    sys.stderr.flush()


def _command(parts, read_response=False):
    sys.stdout.write(" ".join(str(p) for p in parts) + "\n")
    sys.stdout.flush()
    if read_response:
        return sys.stdin.readline().strip()
    return None


# -- queries -----------------------------------------------------------------
def maze_width():
    return int(_command(["mazeWidth"], True))


def maze_height():
    return int(_command(["mazeHeight"], True))


def wall_front():
    return _command(["wallFront"], True) == "true"


def wall_right():
    return _command(["wallRight"], True) == "true"


def wall_left():
    return _command(["wallLeft"], True) == "true"


# -- motion (return ack / crash) ---------------------------------------------
def move_forward(distance=None):
    parts = ["moveForward"]
    if distance is not None:
        parts.append(distance)
    if _command(parts, True) == "crash":
        raise MouseCrashedError()


def turn_right():
    _command(["turnRight"], True)


def turn_left():
    _command(["turnLeft"], True)


# -- display (NO response) ----------------------------------------------------
def set_color(x, y, color):
    _command(["setColor", x, y, color])


def clear_color(x, y):
    _command(["clearColor", x, y])


def clear_all_color():
    _command(["clearAllColor"])


def set_text(x, y, text):
    _command(["setText", x, y, text])


def clear_text(x, y):
    _command(["clearText", x, y])


def clear_all_text():
    _command(["clearAllText"])


def set_wall(x, y, direction):
    """Draw a wall the robot has discovered. `direction` is "n"/"e"/"s"/"w"."""
    _command(["setWall", x, y, direction])


def clear_wall(x, y, direction):
    _command(["clearWall", x, y, direction])


# -- reset handling -----------------------------------------------------------
def was_reset():
    return _command(["wasReset"], True) == "true"


def ack_reset():
    _command(["ackReset"], True)
