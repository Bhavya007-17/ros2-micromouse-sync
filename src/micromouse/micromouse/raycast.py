"""Ray-march a maze to produce laser ranges.

The maze is an axis-aligned grid, so a ray only ever meets a wall at a cell
boundary. That turns ray casting into a grid walk (the classic DDA): step from
one boundary crossing to the next, and at each crossing ask the maze whether
that side of the cell is walled. Exact, and no geometry library needed.

Pure math on a `Maze` -- no ROS, no sensor messages -- so the ranges can be
checked against hand-computed distances.
"""
import math

from . import conventions as C

# Directions crossed when leaving a cell along each axis.
_X_DIR = {True: C.E, False: C.W}    # keyed on "moving in +x"
_Y_DIR = {True: C.N, False: C.S}


def ray_distance(maze, cell_size, x, y, angle, range_max,
                 wall_thickness=0.0):
    """Distance from world point (x, y) along `angle` to the nearest wall face.

    Returns `range_max` if nothing is hit within range. `wall_thickness` shifts
    the answer to the near *face* of the wall rather than its centre line, which
    is where a real beam would stop.
    """
    dx, dy = math.cos(angle), math.sin(angle)
    cx, cy = int(math.floor(x / cell_size)), int(math.floor(y / cell_size))
    if not maze.in_bounds(cx, cy):
        return range_max

    step_x, step_y = (1 if dx > 0 else -1), (1 if dy > 0 else -1)

    def _first_crossing(pos, cell, delta, step):
        if delta == 0.0:
            return math.inf, math.inf
        boundary = (cell + (1 if step > 0 else 0)) * cell_size
        return abs(cell_size / delta), (boundary - pos) / delta

    t_delta_x, t_max_x = _first_crossing(x, cx, dx, step_x)
    t_delta_y, t_max_y = _first_crossing(y, cy, dy, step_y)

    while True:
        if t_max_x < t_max_y:
            t, direction, cx = t_max_x, _X_DIR[dx > 0], cx + step_x
            t_max_x += t_delta_x
            leaving = (cx - step_x, cy)
        else:
            t, direction, cy = t_max_y, _Y_DIR[dy > 0], cy + step_y
            t_max_y += t_delta_y
            leaving = (cx, cy - step_y)
        if t > range_max:
            return range_max
        if maze.has_wall(leaving[0], leaving[1], direction):
            return max(0.0, t - wall_thickness / 2.0)
        if not maze.in_bounds(cx, cy):
            # Should be unreachable on a closed maze; treat as no return.
            return range_max


def scan(maze, cell_size, x, y, yaw, angles, range_max,
         wall_thickness=0.0, noise_stddev=0.0, rng=None):
    """Ranges for a fan of body-frame `angles` from pose (x, y, yaw)."""
    out = []
    for a in angles:
        r = ray_distance(maze, cell_size, x, y, yaw + a, range_max,
                         wall_thickness)
        if noise_stddev and rng is not None and r < range_max:
            r = min(range_max, max(0.0, r + rng.gauss(0.0, noise_stddev)))
        out.append(r)
    return out


def fan_angles(num_rays, angle_min=-math.pi, angle_max=math.pi):
    """Evenly spaced body-frame angles, endpoint excluded so a full circle does
    not sample the same bearing twice."""
    if num_rays < 1:
        return []
    span = angle_max - angle_min
    step = span / num_rays if abs(span - 2 * math.pi) < 1e-9 \
        else (span / (num_rays - 1) if num_rays > 1 else 0.0)
    return [angle_min + i * step for i in range(num_rays)]
