#!/usr/bin/env python3
"""Live 'brain' window -- what the robot believes, next to what is actually there.

Left panel: the belief, rebuilt from `/maze/belief`. Walls the robot has sensed
are solid; walls it has never looked at are faint. Right panel: the real maze,
read from the truth file.

The debug window is allowed to see the truth. The brain is not -- it publishes
its belief and never receives anything back. Watching the left panel fill in and
converge on the right is the whole demonstration.

Falls back to a single truth panel if no belief is being published, so it still
works with the omniscient baseline brain.

Run:  ros2 run micromouse brain_viz
"""
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from geometry_msgs.msg import Point
from std_msgs.msg import String

from micromouse import spec as spec_mod
from micromouse import flood_fill as FF
from micromouse import conventions as C

try:
    import pygame
except ImportError:
    sys.stderr.write("brain_viz needs pygame:  pip install pygame\n")
    raise

# The debug window is allowed to see the truth; the brain is not.
SPEC_PATH = spec_mod.truth_path()
CELL_PX = 26
MARGIN = 20
GAP = 34

UNKNOWN, WALL, OPEN = "?", "#", "."
DIRS = (C.N, C.E, C.S, C.W)


def decode_belief(payload):
    """Parse what explorer_brain publishes: phase|col,row|rows of NESW chars."""
    phase, cell_txt, grid = payload.split("|", 2)
    col, row = (int(v) for v in cell_txt.split(","))
    walls = {}
    for y, line in enumerate(grid.split(";")):
        for x, code in enumerate(line.split(",")):
            walls[(x, y)] = {d: code[i] for i, d in enumerate(DIRS)}
    return phase, (col, row), walls


class VizState:
    def __init__(self):
        self.cur = None
        self.trail = []
        self.belief = None
        self.phase = None
        self.lock = threading.Lock()

    def set_cur(self, col, row):
        with self.lock:
            self.cur = (col, row)
            if not self.trail or self.trail[-1] != (col, row):
                self.trail.append((col, row))

    def set_belief(self, payload):
        try:
            phase, cell, walls = decode_belief(payload)
        except (ValueError, IndexError):
            return          # a malformed frame is not worth killing the window
        with self.lock:
            self.phase, self.belief = phase, walls

    def snapshot(self):
        with self.lock:
            return self.cur, list(self.trail), self.belief, self.phase


class VizNode(Node):
    def __init__(self, state):
        super().__init__("brain_viz")
        self.state = state
        self.create_subscription(Point, "/maze/current_cell", self._on_cell, 10)
        belief_qos = QoSProfile(
            depth=1024,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/maze/belief",
                                 self._on_belief, belief_qos)

    def _on_cell(self, msg: Point):
        self.state.set_cur(int(round(msg.x)), int(round(msg.y)))

    def _on_belief(self, msg: String):
        self.state.set_belief(msg.data)


def _shade(v, vmax):
    if v >= FF.INF:
        return (60, 60, 60)
    f = min(1.0, v / max(1, vmax))
    return (int(205 - 35 * f), int(215 - 75 * f), int(238 - 30 * f))


def run_pygame(node, maze, data, state):
    n = maze.n
    goals = set(tuple(g) for g in data["goal_cells"])
    dist = FF.flood_fill_distances(maze, list(goals))
    vmax = max((v for v in dist.values() if v < FF.INF), default=1)

    pygame.init()
    grid_px = n * CELL_PX
    width = 2 * MARGIN + 2 * grid_px + GAP
    height = grid_px + 2 * MARGIN + 52
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Micromouse -- belief vs truth")
    font = pygame.font.SysFont(None, 15)
    label = pygame.font.SysFont(None, 21)
    clock = pygame.time.Clock()

    def rect(panel, col, row):
        x0 = MARGIN + panel * (grid_px + GAP)
        return pygame.Rect(x0 + col * CELL_PX,
                           MARGIN + (n - 1 - row) * CELL_PX, CELL_PX, CELL_PX)

    def draw_wall(panel, col, row, direction, colour, thickness):
        r = rect(panel, col, row)
        edge = {C.N: (r.topleft, r.topright),
                C.E: (r.topright, r.bottomright),
                C.S: (r.bottomleft, r.bottomright),
                C.W: (r.topleft, r.bottomleft)}[direction]
        pygame.draw.line(screen, colour, edge[0], edge[1], thickness)

    running = True
    while running and rclpy.ok():
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

        cur, trail, belief, phase = state.snapshot()
        screen.fill((245, 245, 245))

        # -- left: the belief ------------------------------------------------
        for col in range(n):
            for row in range(n):
                fill = (200, 235, 200) if (col, row) in goals else (250, 250, 250)
                pygame.draw.rect(screen, fill, rect(0, col, row))
        for (col, row) in trail:
            r = rect(0, col, row).inflate(-int(CELL_PX * 0.6), -int(CELL_PX * 0.6))
            pygame.draw.rect(screen, (90, 140, 240), r, border_radius=2)
        if belief:
            for col in range(n):
                for row in range(n):
                    for d in DIRS:
                        code = belief.get((col, row), {}).get(d, UNKNOWN)
                        if code == WALL:
                            draw_wall(0, col, row, d, (0, 0, 0), 3)
                        elif code == UNKNOWN:
                            draw_wall(0, col, row, d, (222, 222, 222), 1)
        else:
            screen.blit(font.render("waiting for /maze/belief ...", True,
                                    (120, 120, 120)),
                        (MARGIN + 6, MARGIN + 6))
        if cur is not None:
            pygame.draw.rect(screen, (20, 60, 220), rect(0, *cur), 3)

        # -- right: the truth -------------------------------------------------
        for col in range(n):
            for row in range(n):
                r = rect(1, col, row)
                fill = (120, 220, 120) if (col, row) in goals \
                    else _shade(dist[(col, row)], vmax)
                pygame.draw.rect(screen, fill, r)
                t = str(dist[(col, row)]) if dist[(col, row)] < FF.INF else "x"
                screen.blit(font.render(t, True, (25, 25, 25)),
                            font.render(t, True, (25, 25, 25))
                            .get_rect(center=r.center))
        for col in range(n):
            for row in range(n):
                for d in DIRS:
                    if maze.walls[(col, row)][C.DIR_KEYS[d]]:
                        draw_wall(1, col, row, d, (0, 0, 0), 3)
        if cur is not None:
            pygame.draw.rect(screen, (20, 60, 220), rect(1, *cur), 3)

        # -- captions ----------------------------------------------------------
        base = MARGIN + grid_px + 10
        left_caption = "BELIEF -- only what the robot has sensed"
        if phase:
            left_caption += "   [phase: %s]" % phase
        screen.blit(label.render(left_caption, True, (40, 40, 40)),
                    (MARGIN, base))
        screen.blit(label.render("TRUTH -- the maze as built", True, (40, 40, 40)),
                    (MARGIN + grid_px + GAP, base))
        screen.blit(font.render(
            "solid = sensed wall   faint = never looked   blue = path taken",
            True, (110, 110, 110)), (MARGIN, base + 22))

        pygame.display.flip()
        rclpy.spin_once(node, timeout_sec=0.0)
        clock.tick(30)

    pygame.quit()


def main(argv=None):
    rclpy.init(args=argv)
    state = VizState()
    node = VizNode(state)
    maze, data = spec_mod.read_truth(SPEC_PATH)
    try:
        run_pygame(node, maze, data, state)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
