#!/usr/bin/env python3
"""Record the explorer discovering a maze, as a GIF.

Every frame is one decision: the belief on the left as the robot has sensed it,
the real maze on the right. Watching the left panel fill in and then produce a
single clean line on the speed run is the whole point of the project, so the
frames come straight from `/maze/belief` rather than being re-simulated.

The recorder is allowed to read the truth file. The brain is not -- it publishes
its belief and receives nothing back.

    python3 scripts/record_exploration_gif.py --out media/exploration.gif

Start it before (or during) a run; `/maze/belief` is TRANSIENT_LOCAL with deep
history, so a late subscriber replays everything it missed.
"""
import argparse
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "micromouse"))

import rclpy                                              # noqa: E402
from rclpy.node import Node                               # noqa: E402
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,   # noqa: E402
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import String                           # noqa: E402
from PIL import Image, ImageDraw                          # noqa: E402

from micromouse import conventions as C                   # noqa: E402
from micromouse import spec as spec_mod                   # noqa: E402
from micromouse.brain_viz import decode_belief            # noqa: E402

CELL = 24
MARGIN = 14
GAP = 26
CAPTION = 34

BG = (247, 247, 248)
PANEL = (255, 255, 255)
GOAL = (188, 232, 190)
SENSED = (24, 24, 28)
UNSEEN = (226, 226, 230)
TRUTH_WALL = (24, 24, 28)
TRUTH_FILL = (236, 240, 246)
ROBOT = (28, 82, 224)
PHASE_TINT = {"search": (150, 186, 250), "return": (247, 205, 120),
              "speed": (104, 200, 130), "done": (104, 200, 130)}


class BeliefRecorder(Node):
    def __init__(self):
        super().__init__("exploration_recorder")
        self.frames = []
        self.last = 0.0
        self.lock = threading.Lock()
        qos = QoSProfile(depth=4096,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/maze/belief", self._on_belief, qos)

    def _on_belief(self, msg: String):
        try:
            frame = decode_belief(msg.data)
        except (ValueError, IndexError):
            return
        with self.lock:
            self.frames.append(frame)
            self.last = time.time()

    def snapshot(self):
        with self.lock:
            return list(self.frames), self.last


def _panel_origin(index, grid_px):
    return MARGIN + index * (grid_px + GAP), MARGIN


def _cell_box(ox, oy, n, col, row):
    x = ox + col * CELL
    y = oy + (n - 1 - row) * CELL
    return x, y, x + CELL, y + CELL


def _wall_line(box, direction):
    x0, y0, x1, y1 = box
    return {C.N: (x0, y0, x1, y0), C.S: (x0, y1, x1, y1),
            C.E: (x1, y0, x1, y1), C.W: (x0, y0, x0, y1)}[direction]


def render(frame, maze, goals, trail):
    phase, cur, belief = frame
    n = maze.n
    grid = n * CELL
    width = 2 * MARGIN + 2 * grid + GAP
    height = 2 * MARGIN + grid + CAPTION
    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)

    bx, by = _panel_origin(0, grid)
    tx, ty = _panel_origin(1, grid)
    d.rectangle([bx, by, bx + grid, by + grid], fill=PANEL)
    d.rectangle([tx, ty, tx + grid, ty + grid], fill=PANEL)

    for (col, row) in goals:
        d.rectangle(_cell_box(bx, by, n, col, row), fill=GOAL)
        d.rectangle(_cell_box(tx, ty, n, col, row), fill=GOAL)

    tint = PHASE_TINT.get(phase, ROBOT)
    for (col, row) in trail:
        x0, y0, x1, y1 = _cell_box(bx, by, n, col, row)
        pad = CELL // 3
        d.rectangle([x0 + pad, y0 + pad, x1 - pad, y1 - pad], fill=tint)

    # Belief: unknown walls faint, sensed walls solid.
    for col in range(n):
        for row in range(n):
            box = _cell_box(bx, by, n, col, row)
            for direction in (C.N, C.E, C.S, C.W):
                code = belief.get((col, row), {}).get(direction, "?")
                if code == "#":
                    d.line(_wall_line(box, direction), fill=SENSED, width=3)
                elif code == "?":
                    d.line(_wall_line(box, direction), fill=UNSEEN, width=1)

    # Truth.
    for col in range(n):
        for row in range(n):
            box = _cell_box(tx, ty, n, col, row)
            if (col, row) not in goals:
                d.rectangle(box, fill=TRUTH_FILL)
            for direction in (C.N, C.E, C.S, C.W):
                if maze.walls[(col, row)][C.DIR_KEYS[direction]]:
                    d.line(_wall_line(box, direction), fill=TRUTH_WALL, width=3)

    for ox, oy in ((bx, by), (tx, ty)):
        d.rectangle(_cell_box(ox, oy, n, *cur), outline=ROBOT, width=3)

    cap_y = by + grid + 10
    d.text((bx, cap_y), "BELIEF  -  phase: %s" % phase, fill=(40, 40, 44))
    d.text((tx, cap_y), "TRUTH", fill=(40, 40, 44))
    d.text((bx, cap_y + 14),
           "solid = sensed wall    faint = never looked",
           fill=(130, 130, 136))
    return img


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="media/exploration.gif")
    p.add_argument("--duration", type=float, default=600.0,
                   help="max seconds to wait for frames")
    p.add_argument("--settle", type=float, default=6.0,
                   help="stop this long after the last frame arrives")
    p.add_argument("--frame-ms", type=int, default=90)
    p.add_argument("--hold", type=int, default=25,
                   help="extra frames held on the finished map")
    p.add_argument("--every", type=int, default=1,
                   help="keep every Nth frame (thins long runs)")
    args = p.parse_args(argv)

    maze, data = spec_mod.read_truth(spec_mod.truth_path())
    goals = {tuple(g) for g in data["goal_cells"]}

    rclpy.init()
    node = BeliefRecorder()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("collecting belief frames...", flush=True)
    deadline = time.time() + args.duration
    while time.time() < deadline:
        frames, last = node.snapshot()
        if frames and time.time() - last > args.settle:
            break
        time.sleep(0.25)
    frames, _ = node.snapshot()
    rclpy.shutdown()

    if not frames:
        print("no frames on /maze/belief -- was the explorer running?")
        return 1
    print("collected %d frames; rendering..." % len(frames), flush=True)

    # Render offline, one pass, so the executor is never starved by drawing.
    images, trail, seen_phase = [], [], None
    for i, frame in enumerate(frames):
        phase, cur = frame[0], frame[1]
        # Each phase draws its own path. Carrying the trail across would paint
        # the search run's wandering in the speed run's colour and hide the one
        # thing worth seeing: the speed run is a single clean line.
        if phase != seen_phase:
            trail, seen_phase = [], phase
        if not trail or trail[-1] != cur:
            trail.append(cur)
        if i % args.every == 0 or i == len(frames) - 1:
            images.append(render(frame, maze, goals, list(trail)))

    images.extend([images[-1]] * args.hold)
    out = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    images[0].save(out, save_all=True, append_images=images[1:],
                   duration=args.frame_ms, loop=0, optimize=True)
    print("wrote %s (%d frames, %.1f MB)"
          % (out, len(images), os.path.getsize(out) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
