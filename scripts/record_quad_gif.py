#!/usr/bin/env python3
"""Four views of the same run, side by side.

    BELIEF          TRUTH
    GAZEBO          RVIZ

Belief and truth are drawn from `/maze/belief`. Gazebo and RViz are real
windows, captured off nested virtual displays -- WSLg is rootless, so there is
no X root to grab and the apps have to be given their own Xvfb screens (see
scripts/record_quad_session.sh).

Two steps, because the run has to happen in between:

    record_quad_gif.py collect  --dir /tmp/quad          # during the run
    record_quad_gif.py compose  --dir /tmp/quad \\
        --gz /tmp/quad/gz.mkv --rviz /tmp/quad/rviz.mkv --out media/quad.gif

Frames are paired by wall-clock time: for each output tick, each source
contributes the most recent frame it had produced by then. The sources run at
different rates -- software-rendered Gazebo is far slower than the belief
stream -- so this is a resample, not a strict interleave.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "micromouse"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image, ImageDraw                          # noqa: E402

from micromouse import conventions as C                   # noqa: E402
from micromouse import spec as spec_mod                   # noqa: E402

PANEL = 360
LABEL = 26
BG = (247, 247, 248)
INK = (34, 34, 38)
SUB = (128, 128, 134)


# -- panel drawing ----------------------------------------------------------
def _grid_metrics(n):
    inner = PANEL - 24
    cell = inner // n
    span = cell * n
    off = (PANEL - span) // 2
    return cell, span, off


def _box(n, cell, off, col, row):
    x = off + col * cell
    y = off + (n - 1 - row) * cell
    return x, y, x + cell, y + cell


def _edge(box, direction):
    x0, y0, x1, y1 = box
    return {C.N: (x0, y0, x1, y0), C.S: (x0, y1, x1, y1),
            C.E: (x1, y0, x1, y1), C.W: (x0, y0, x0, y1)}[direction]


def _blank(n):
    img = Image.new("RGB", (PANEL, PANEL), (255, 255, 255))
    return img, ImageDraw.Draw(img), _grid_metrics(n)


def draw_belief(n, goals, belief, cur, trail, tint):
    img, d, (cell, _, off) = _blank(n)
    for g in goals:
        d.rectangle(_box(n, cell, off, *g), fill=(188, 232, 190))
    pad = max(2, cell // 3)
    for (col, row) in trail:
        x0, y0, x1, y1 = _box(n, cell, off, col, row)
        d.rectangle([x0 + pad, y0 + pad, x1 - pad, y1 - pad], fill=tint)
    for col in range(n):
        for row in range(n):
            box = _box(n, cell, off, col, row)
            for direction in (C.N, C.E, C.S, C.W):
                code = belief.get((col, row), {}).get(direction, "?")
                if code == "#":
                    d.line(_edge(box, direction), fill=INK, width=3)
                elif code == "?":
                    d.line(_edge(box, direction), fill=(228, 228, 232), width=1)
    if cur:
        d.rectangle(_box(n, cell, off, *cur), outline=(28, 82, 224), width=3)
    return img


def draw_truth(maze, goals, cur):
    n = maze.n
    img, d, (cell, _, off) = _blank(n)
    for col in range(n):
        for row in range(n):
            box = _box(n, cell, off, col, row)
            d.rectangle(box, fill=(188, 232, 190) if (col, row) in goals
                        else (236, 240, 246))
            for direction in (C.N, C.E, C.S, C.W):
                if maze.walls[(col, row)][C.DIR_KEYS[direction]]:
                    d.line(_edge(box, direction), fill=INK, width=3)
    if cur:
        d.rectangle(_box(n, cell, off, *cur), outline=(28, 82, 224), width=3)
    return img


def placeholder(text):
    img = Image.new("RGB", (PANEL, PANEL), (250, 250, 251))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, PANEL - 1, PANEL - 1], outline=(225, 225, 229))
    d.text((16, PANEL // 2), text, fill=SUB)
    return img


def fit(img):
    """Letterbox any captured frame into the panel square."""
    img = img.convert("RGB")
    img.thumbnail((PANEL, PANEL), Image.LANCZOS)
    canvas = Image.new("RGB", (PANEL, PANEL), (255, 255, 255))
    canvas.paste(img, ((PANEL - img.width) // 2, (PANEL - img.height) // 2))
    return canvas


def quad(panels, captions, phase):
    w = 2 * PANEL + 18
    h = 2 * (PANEL + LABEL) + 24
    img = Image.new("RGB", (w, h), BG)
    d = ImageDraw.Draw(img)
    for i, (panel, caption) in enumerate(zip(panels, captions)):
        x = 6 + (i % 2) * (PANEL + 6)
        y = 6 + (i // 2) * (PANEL + LABEL + 6)
        img.paste(panel, (x, y))
        d.text((x + 2, y + PANEL + 6), caption, fill=INK)
    d.text((8, h - 14), "phase: %s" % phase, fill=SUB)
    return img


# -- collection --------------------------------------------------------------
def collect(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from std_msgs.msg import String

    class Collector(Node):
        def __init__(self):
            super().__init__("quad_collector")
            self.rows = []
            self.last = 0.0
            self.lock = threading.Lock()
            qos = QoSProfile(depth=4096,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/maze/belief", self._on, qos)

        def _on(self, msg):
            with self.lock:
                self.rows.append([time.time(), msg.data])
                self.last = time.time()

        def snap(self):
            with self.lock:
                return list(self.rows), self.last

    os.makedirs(args.dir, exist_ok=True)
    rclpy.init()
    node = Collector()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("collecting belief frames...", flush=True)
    deadline = time.time() + args.duration
    while time.time() < deadline:
        rows, last = node.snap()
        if rows and time.time() - last > args.settle:
            break
        time.sleep(0.25)
    rows, _ = node.snap()
    rclpy.shutdown()

    with open(os.path.join(args.dir, "belief.json"), "w") as f:
        json.dump(rows, f)
    print("wrote %d belief frames" % len(rows), flush=True)
    return 0 if rows else 1


# -- composition -------------------------------------------------------------
def extract(video, out_dir, fps):
    """Explode a capture into numbered PNGs. Returns them in order."""
    if not video or not os.path.exists(video):
        return []
    os.makedirs(out_dir, exist_ok=True)
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", "-i", video,
         "-vf", "fps=%g" % fps, os.path.join(out_dir, "f_%05d.png")],
        check=False)
    return sorted(os.path.join(out_dir, f) for f in os.listdir(out_dir)
                  if f.endswith(".png"))


def pick(frames, index):
    if not frames:
        return None
    return frames[min(index, len(frames) - 1)]


def compose(args):
    from record_exploration_gif import PHASE_TINT
    from micromouse.brain_viz import decode_belief

    maze, data = spec_mod.read_truth(spec_mod.truth_path())
    goals = {tuple(g) for g in data["goal_cells"]}
    n = maze.n

    with open(os.path.join(args.dir, "belief.json")) as f:
        rows = json.load(f)
    if not rows:
        print("no belief frames collected")
        return 1

    gz = extract(args.gz, os.path.join(args.dir, "gz"), args.fps)
    rviz = extract(args.rviz, os.path.join(args.dir, "rviz"), args.fps)
    print("belief=%d gazebo=%d rviz=%d frames"
          % (len(rows), len(gz), len(rviz)), flush=True)

    t0, t1 = rows[0][0], rows[-1][0]
    ticks = max(1, int((t1 - t0) * args.fps))
    # The video captures started with the run, so their frame index maps
    # directly onto the tick; belief frames are looked up by timestamp.
    images, cursor, trail, seen_phase = [], 0, [], None

    for tick in range(0, ticks, max(1, args.every)):
        now = t0 + tick / float(args.fps)
        while cursor + 1 < len(rows) and rows[cursor + 1][0] <= now:
            cursor += 1
        try:
            phase, cur, belief = decode_belief(rows[cursor][1])
        except (ValueError, IndexError):
            continue
        if phase != seen_phase:
            trail, seen_phase = [], phase
        if not trail or trail[-1] != cur:
            trail.append(cur)

        gz_path, rviz_path = pick(gz, tick), pick(rviz, tick)
        panels = [
            draw_belief(n, goals, belief, cur, list(trail),
                        PHASE_TINT.get(phase, (150, 186, 250))),
            draw_truth(maze, goals, cur),
            fit(Image.open(gz_path)) if gz_path
            else placeholder("Gazebo capture missing"),
            fit(Image.open(rviz_path)) if rviz_path
            else placeholder("RViz capture missing"),
        ]
        images.append(quad(panels, [
            "BELIEF  -  only what the robot has sensed",
            "TRUTH  -  the maze as built",
            "GAZEBO  -  the robot in the physical world",
            "RVIZ  -  live /scan and odometry",
        ], phase))

    if not images:
        print("nothing to render")
        return 1
    images.extend([images[-1]] * args.hold)
    out = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    images[0].save(out, save_all=True, append_images=images[1:],
                   duration=args.frame_ms, loop=0, optimize=True)
    print("wrote %s (%d frames, %.1f MB)"
          % (out, len(images), os.path.getsize(out) / 1e6))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("collect")
    c.add_argument("--dir", required=True)
    c.add_argument("--duration", type=float, default=1800.0)
    c.add_argument("--settle", type=float, default=8.0)
    c.set_defaults(func=collect)

    m = sub.add_parser("compose")
    m.add_argument("--dir", required=True)
    m.add_argument("--gz")
    m.add_argument("--rviz")
    m.add_argument("--out", default="media/quad.gif")
    m.add_argument("--fps", type=float, default=4.0)
    m.add_argument("--every", type=int, default=1)
    m.add_argument("--frame-ms", type=int, default=110)
    m.add_argument("--hold", type=int, default=20)
    m.set_defaults(func=compose)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
