"""The machine-readable contract shared by every component.

The maze is written out as TWO files so that "the brain solves an unseen maze"
is a property of the architecture rather than a promise:

  maze_spec.json  (public) -- size, cell geometry, start, goal. NO walls.
  maze_truth.json (truth)  -- the same plus every wall.

The explorer brain reads only the public file. The world builder, the mms .num
writer, the simulated LiDAR and the debug viz read the truth file. `read_public`
refuses a file that contains walls, so mis-wiring a component to the truth file
fails loudly instead of quietly handing it the answer.

`to_dict`/`write`/`read` are the original single-file API. They are kept for
`gazebo_sync_brain`, the omniscient baseline the explorer is measured against.
"""
import json
import os

from .maze import Maze
from . import conventions as C

PUBLIC_NAME = "maze_spec.json"
TRUTH_NAME = "maze_truth.json"
DEFAULT_DIR = os.path.expanduser("~/.micromouse")

PUBLIC_KEYS = ("n", "cell_size_m", "wall_thickness_m", "wall_height_m",
               "start_cell", "goal_cells")


def public_path() -> str:
    """Where the public spec lives ($MICROMOUSE_SPEC, else the default dir)."""
    return os.environ.get("MICROMOUSE_SPEC",
                          os.path.join(DEFAULT_DIR, PUBLIC_NAME))


def truth_path() -> str:
    """Where the truth file lives.

    Falls back to a sibling of the public spec, so pointing one component at a
    custom maze directory moves both files without a second env var.
    """
    explicit = os.environ.get("MICROMOUSE_TRUTH")
    if explicit:
        return explicit
    return os.path.join(os.path.dirname(public_path()), TRUTH_NAME)


def public_dict(maze: Maze, cell_size: float, wall_thickness: float,
                wall_height: float) -> dict:
    """Everything a blind robot is allowed to know before it starts."""
    return {
        "n": maze.n,
        "cell_size_m": cell_size,
        "wall_thickness_m": wall_thickness,
        "wall_height_m": wall_height,
        "start_cell": list(C.start_cell()),
        "goal_cells": [list(g) for g in C.goal_cells(maze.n)],
    }


def to_dict(maze: Maze, cell_size: float, wall_thickness: float,
            wall_height: float) -> dict:
    """The public fields plus the walls."""
    walls = {}
    for (x, y), w in maze.walls.items():
        walls["{},{}".format(x, y)] = {
            "N": bool(w["N"]), "E": bool(w["E"]),
            "S": bool(w["S"]), "W": bool(w["W"]),
        }
    data = public_dict(maze, cell_size, wall_thickness, wall_height)
    data["walls"] = walls
    return data


truth_dict = to_dict


def _dump(path: str, data: dict):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def write_public(path: str, maze: Maze, cell_size: float,
                 wall_thickness: float, wall_height: float):
    _dump(path, public_dict(maze, cell_size, wall_thickness, wall_height))


def write_truth(path: str, maze: Maze, cell_size: float,
                wall_thickness: float, wall_height: float):
    _dump(path, to_dict(maze, cell_size, wall_thickness, wall_height))


def write(path: str, maze: Maze, cell_size: float, wall_thickness: float,
          wall_height: float):
    """Legacy single-file writer (same content as `write_truth`)."""
    write_truth(path, maze, cell_size, wall_thickness, wall_height)


def read_public(path: str) -> dict:
    """Load the public spec. Raises if the file carries walls.

    This is the anti-cheat boundary: a component that reads through here cannot
    be handed the maze even by accident.
    """
    with open(path) as f:
        data = json.load(f)
    if "walls" in data:
        raise ValueError(
            "%s contains walls -- this is the truth file, not the public spec. "
            "A blind component must not read it." % path)
    missing = [k for k in PUBLIC_KEYS if k not in data]
    if missing:
        raise ValueError("%s is missing public spec keys: %s"
                         % (path, ", ".join(missing)))
    return data


def _maze_from(data: dict) -> Maze:
    maze = Maze(data["n"])
    for key, w in data["walls"].items():
        x, y = (int(v) for v in key.split(","))
        maze.walls[(x, y)] = {
            "N": bool(w["N"]), "E": bool(w["E"]),
            "S": bool(w["S"]), "W": bool(w["W"]),
        }
    return maze


def read_truth(path: str):
    """Load the truth file and rebuild a Maze plus the metadata dict."""
    with open(path) as f:
        data = json.load(f)
    if "walls" not in data:
        raise ValueError("%s has no walls -- this is the public spec, not the "
                         "truth file." % path)
    return _maze_from(data), data


def read(path: str):
    """Legacy single-file reader (same as `read_truth`)."""
    return read_truth(path)
