"""Navigation decisions, kept free of mms/ROS so they can be unit-tested.

With the full maze known, flood-fill distances form a gradient with no local
minima, so always stepping to the lowest reachable neighbor walks straight to
the goal.
"""
from . import conventions as C


def pick_next_dir(maze, dist, cur, heading, visited=None):
    """Return the best direction to step from `cur`, or None if already a sink.

    Chooses the reachable neighbor with the smallest distance. Ties are broken
    in this order:

      1. an unvisited cell, when `visited` is supplied -- during exploration
         this steers the robot into new ground at no cost, because only
         equal-distance candidates ever reach this test
      2. continuing straight (current heading), which keeps the path smooth
      3. the fixed order N, E, S, W, which keeps the behavior deterministic

    Passing `visited=None` reproduces the original two-rule behavior exactly.
    """
    best_key = None
    best_dir = None
    for d in (C.N, C.E, C.S, C.W):
        if not maze.open_between(cur[0], cur[1], d):
            continue
        nxt = maze.neighbor(cur[0], cur[1], d)
        key = (dist[nxt],
               0 if (visited is not None and nxt not in visited) else 1,
               0 if d == heading else 1)
        if best_key is None or key < best_key:
            best_key, best_dir = key, d
    if best_dir is None or best_key[0] >= dist[cur]:
        return None  # no strictly-better neighbor (we are at/below the goal)
    return best_dir


def turns_for(heading, target_dir):
    """Minimal sequence of 'turnLeft'/'turnRight' to face `target_dir`."""
    delta = (target_dir - heading) % 4
    if delta == 0:
        return []
    if delta == 1:
        return ["turnRight"]
    if delta == 2:
        return ["turnRight", "turnRight"]
    return ["turnLeft"]  # delta == 3


def apply_turn(heading, cmd):
    return C.turn_right(heading) if cmd == "turnRight" else C.turn_left(heading)
