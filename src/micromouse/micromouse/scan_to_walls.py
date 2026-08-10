"""Read three wall booleans out of a laser scan.

This is the robot's entire sense of the world. It takes `(angle, range)` pairs in
the body frame -- 0 rad straight ahead, positive counter-clockwise, the ROS
convention -- and answers: is there a wall in front, to the left, to the right?

Deliberately free of ROS message types. The node hands over plain numbers, which
means the interpretation can be tested with hand-written scans instead of a
running simulator.

Why the threshold works: from the centre of a cell, an adjacent wall face sits at
about `cell/2` (0.14 m for a 0.30 m cell) while the nearest wall through an open
side is a full cell further away (about 0.44 m). Anything under three quarters of
a cell is therefore a wall, with a wide margin on both sides.
"""
import math
import statistics

from . import conventions as C

FRONT, LEFT, RIGHT = "front", "left", "right"

# Bearing of each sector in the body frame.
SECTORS = {
    FRONT: 0.0,
    LEFT: math.pi / 2,
    RIGHT: -math.pi / 2,
}

DEFAULT_WINDOW_RAD = 0.15      # +/- ~8.5 degrees around each bearing
DEFAULT_THRESHOLD_FRAC = 0.75  # of one cell pitch


def samples_from_ranges(ranges, angle_min, angle_increment):
    """Zip a LaserScan's `ranges` array with the angles it implies."""
    return [(angle_min + i * angle_increment, r)
            for i, r in enumerate(ranges)]


def _finite(value, far):
    """Map a no-return (inf/nan/None) onto `far`, so it reads as open."""
    if value is None:
        return far
    try:
        v = float(value)
    except (TypeError, ValueError):
        return far
    if math.isnan(v) or math.isinf(v):
        return far
    return v


def walls_from_scan(samples, cell_size, window_rad=DEFAULT_WINDOW_RAD,
                    threshold_frac=DEFAULT_THRESHOLD_FRAC, range_max=None):
    """Return `{"front": bool|None, "left": ..., "right": ...}`.

    `None` means no beam covered that sector -- unknown, which is not the same
    as open, and the caller must not record it as a passage.

    The median over each sector's beams is what makes this survive a noisy
    LiDAR: a single bad return cannot move it.
    """
    threshold = cell_size * threshold_frac
    far = range_max if range_max is not None else cell_size * 100.0

    buckets = {name: [] for name in SECTORS}
    for angle, value in samples:
        for name, bearing in SECTORS.items():
            if abs(C.normalize_angle(angle - bearing)) <= window_rad:
                r = _finite(value, far)
                buckets[name].append(min(r, far) if range_max else r)
                break

    return {name: (statistics.median(vals) < threshold if vals else None)
            for name, vals in buckets.items()}


def to_absolute(relative, cell, heading):
    """Convert relative readings into `learn_many` updates for `BeliefMaze`.

    `relative` is what `walls_from_scan` returned; sectors reading `None` are
    dropped rather than guessed.
    """
    facing = {
        FRONT: heading,
        LEFT: C.turn_left(heading),
        RIGHT: C.turn_right(heading),
    }
    x, y = cell
    return [(x, y, facing[name], bool(is_wall))
            for name, is_wall in relative.items()
            if is_wall is not None]
