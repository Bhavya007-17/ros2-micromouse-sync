#!/usr/bin/env python3
"""Record live solve GIFs from ROS topics during a Gazebo + brain run.

Subscribes to /maze/current_cell and builds two animations:
  * media/brain_path.gif   — flood-fill grid with trail (brain window style)
  * media/gazebo_path.gif  — top-down world view with robot icon

Run in a sourced ROS terminal while maze_gazebo.launch.py and the brain are up:

    python3 scripts/record_session_gifs.py --duration 120
"""
import argparse
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "micromouse"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from geometry_msgs.msg import Point

from micromouse import conventions as C
from micromouse import flood_fill as FF
from micromouse import spec as spec_mod


class Recorder(Node):
    def __init__(self, state):
        super().__init__("session_gif_recorder")
        self.state = state
        # Match the brain's TRANSIENT_LOCAL trail publisher so we replay every
        # cell even though we join after the sub-second burst has finished.
        qos = QoSProfile(
            depth=1024,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Point, "/maze/current_cell", self._on_cell, qos)

    def _on_cell(self, msg: Point):
        col, row = int(round(msg.x)), int(round(msg.y))
        with self.state["lock"]:
            if not self.state["trail"] or self.state["trail"][-1] != (col, row):
                self.state["trail"].append((col, row))
            self.state["cur"] = (col, row)
            self.state["last_update"] = time.time()


def _brain_frame(maze, dist, goals, trail, cur, vmax, cell_px=28):
    n = maze.n
    margin = 20
    w = n * cell_px + 2 * margin
    h = w + 30
    fig, ax = plt.subplots(figsize=(w / 80, h / 80), dpi=80)
    ax.set_xlim(-0.05, n - 0.05)
    ax.set_ylim(-0.05, n - 0.05)
    ax.set_aspect("equal")
    ax.axis("off")

    for col in range(n):
        for row in range(n):
            val = dist[(col, row)]
            color = "#78dc78" if (col, row) in goals else plt.cm.Blues(
                1.0 - min(1.0, val / max(1, vmax)))
            ax.add_patch(mpatches.Rectangle((col, row), 1, 1,
                                            facecolor=color, edgecolor="none"))
            ax.text(col + 0.5, row + 0.5,
                    str(val) if val < FF.INF else "x",
                    ha="center", va="center", fontsize=6)

    for c, r in trail:
        ax.add_patch(mpatches.Rectangle(
            (c + 0.2, r + 0.2), 0.6, 0.6, facecolor="#5a8ce0", edgecolor="none"))
    if cur:
        ax.add_patch(mpatches.Rectangle(
            (cur[0], cur[1]), 1, 1, fill=False, edgecolor="#143cdc", lw=2.5))

    for col in range(n):
        for row in range(n):
            wll = maze.walls[(col, row)]
            if wll["N"]:
                ax.plot([col, col + 1], [row, row], "k", lw=1.5)
            if wll["S"]:
                ax.plot([col, col + 1], [row + 1, row + 1], "k", lw=1.5)
            if wll["E"]:
                ax.plot([col + 1, col + 1], [row, row + 1], "k", lw=1.5)
            if wll["W"]:
                ax.plot([col, col], [row, row + 1], "k", lw=1.5)

    fig.canvas.draw()
    img = Image.frombytes(
        "RGB", fig.canvas.get_width_height(),
        fig.canvas.tostring_rgb())
    plt.close(fig)
    return img


def _gazebo_frame(maze, cell_m, trail, cur):
    span = maze.n * cell_m
    fig, ax = plt.subplots(figsize=(6.5, 6.5), dpi=90)
    ax.set_xlim(0, span)
    ax.set_ylim(0, span)
    ax.set_aspect("equal")
    ax.set_facecolor("#e8e0d0")
    ax.set_title("Gazebo overhead — robot path", fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    for col in range(maze.n):
        for row in range(maze.n):
            x0 = col * cell_m
            y0 = row * cell_m
            wll = maze.walls[(col, row)]
            if wll["N"]:
                ax.plot([x0, x0 + cell_m], [y0 + cell_m, y0 + cell_m], "#333", lw=2)
            if wll["S"]:
                ax.plot([x0, x0 + cell_m], [y0, y0], "#333", lw=2)
            if wll["E"]:
                ax.plot([x0 + cell_m, x0 + cell_m], [y0, y0 + cell_m], "#333", lw=2)
            if wll["W"]:
                ax.plot([x0, x0], [y0, y0 + cell_m], "#333", lw=2)

    if len(trail) > 1:
        xs = [C.cell_to_world(c, r, cell_m)[0] for c, r in trail]
        ys = [C.cell_to_world(c, r, cell_m)[1] for c, r in trail]
        ax.plot(xs, ys, color="#2563eb", lw=2.5, alpha=0.7)

    if cur:
        cx, cy = C.cell_to_world(*cur, cell_m)
        ax.add_patch(mpatches.Rectangle(
            (cx - cell_m * 0.18, cy - cell_m * 0.12),
            cell_m * 0.36, cell_m * 0.24,
            angle=90, facecolor="#dc2626", edgecolor="#7f1d1d", lw=1.5))

    fig.canvas.draw()
    img = Image.frombytes(
        "RGB", fig.canvas.get_width_height(),
        fig.canvas.tostring_rgb())
    plt.close(fig)
    return img


def main(argv=None):
    p = argparse.ArgumentParser()
    # The recorder draws the real maze, so it reads the truth file.
    p.add_argument("--spec", default=spec_mod.truth_path())
    p.add_argument("--duration", type=float, default=90.0,
                   help="max seconds to wait for the trail to arrive")
    p.add_argument("--settle", type=float, default=2.5,
                   help="stop collecting this many seconds after the last cell")
    p.add_argument("--interval", type=float, default=0.12,
                   help="GIF frame duration in seconds")
    p.add_argument("--hold", type=int, default=12,
                   help="extra frames to hold on the final solved path")
    p.add_argument("--brain-gif", default="media/brain_path.gif")
    p.add_argument("--gazebo-gif", default="media/gazebo_path.gif")
    args = p.parse_args(argv)

    maze, data = spec_mod.read_truth(args.spec)
    goals = set(tuple(g) for g in data["goal_cells"])
    dist = FF.flood_fill_distances(maze, list(goals))
    vmax = max((v for v in dist.values() if v < FF.INF), default=1)
    cell_m = float(data["cell_size_m"])

    state = {"trail": [tuple(data["start_cell"])], "cur": tuple(data["start_cell"]),
             "lock": threading.Lock(), "last_update": 0.0}

    rclpy.init()
    node = Recorder(state)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    # --- Phase 1: COLLECT the full ordered trail -------------------------------
    # The brain publishes the entire path in a sub-second TRANSIENT_LOCAL burst;
    # we just wait for it to arrive and settle. Collecting before rendering keeps
    # the rclpy executor from being starved by matplotlib (the bug that produced
    # an empty, single-frame GIF).
    print("collecting trail (max %.0fs, settle %.1fs)..."
          % (args.duration, args.settle), flush=True)
    deadline = time.time() + args.duration
    while time.time() < deadline:
        with state["lock"]:
            n_cells = len(state["trail"])
            last = state["last_update"]
        if n_cells > 1 and last and (time.time() - last) > args.settle:
            break
        time.sleep(0.1)

    with state["lock"]:
        trail = list(state["trail"])
    print("collected %d trail cells" % len(trail), flush=True)

    # --- Phase 2: RENDER a progressive reveal of the collected trail -----------
    # One frame per cell guarantees a smooth animation regardless of how fast the
    # cells actually arrived over the network.
    os.makedirs("media", exist_ok=True)
    brain_frames, gazebo_frames = [], []
    for k in range(1, len(trail) + 1):
        partial = trail[:k]
        cur = partial[-1]
        brain_frames.append(_brain_frame(maze, dist, goals, partial, cur, vmax))
        gazebo_frames.append(_gazebo_frame(maze, cell_m, partial, cur))
    # Hold on the final solved frame so the loop pauses on the answer.
    if brain_frames:
        brain_frames += [brain_frames[-1]] * args.hold
        gazebo_frames += [gazebo_frames[-1]] * args.hold

    dur_ms = int(args.interval * 1000)
    if brain_frames:
        brain_frames[0].save(
            args.brain_gif, save_all=True, append_images=brain_frames[1:],
            duration=dur_ms, loop=0)
        print("wrote", args.brain_gif, "(%d frames)" % len(brain_frames))
    if gazebo_frames:
        gazebo_frames[0].save(
            args.gazebo_gif, save_all=True, append_images=gazebo_frames[1:],
            duration=dur_ms, loop=0)
        print("wrote", args.gazebo_gif, "(%d frames)" % len(gazebo_frames))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
