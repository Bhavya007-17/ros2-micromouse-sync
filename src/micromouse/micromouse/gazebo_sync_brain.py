#!/usr/bin/env python3
"""Digital vision brain -- the mms algorithm that also drives the Gazebo robot.

Run by the mms GUI as the Mouse "Run command". Each step it:
  1. reproduces the mms idle display (a flood-fill distance in every cell),
  2. picks the lowest-distance reachable neighbor,
  3. publishes the move on /maze/mms_command AND animates the mms mouse,
  4. publishes the new cell on /maze/current_cell so the live brain window
     fills the trail immediately.
The mms mouse and the trail window run to the center as fast as the algorithm
can think -- they do NOT wait for the robot. The controller buffers the commands
and animates the Gazebo robot one cell at a time, catching up smoothly. Only at
the very end do we wait for the robot to drain its queue (so mms keeps the
process alive and no command is lost).

Full maze is known up front (read from maze_truth.json), so no wall-sensing and
no re-flooding -- distances are computed once, exactly like the screenshot.

This is the OMNISCIENT BASELINE. It is kept deliberately: `explorer_brain.py`
solves the same maze without ever seeing it, and the contrast between the two is
the point. If you want the robot to actually discover the maze, run that one.

IMPORTANT: stdout is the mms protocol channel. All diagnostics go to stderr.
"""
import os
import sys
import threading
import time

# Make the package importable whether mms runs this as a file or a module.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

from micromouse import conventions as C
from micromouse import flood_fill as FF
from micromouse import planner as P
from micromouse import spec as spec_mod
from micromouse import mms_interface as mms

SPEC_PATH = spec_mod.truth_path()
CMD_TOPIC = os.environ.get("MICROMOUSE_CMD_TOPIC", "/maze/mms_command")
DONE_TOPIC = os.environ.get("MICROMOUSE_DONE_TOPIC", "/maze/step_complete")
CELL_TOPIC = os.environ.get("MICROMOUSE_CELL_TOPIC", "/maze/current_cell")
# Overall budget for the robot to finish the whole path after mms is done.
STEP_TIMEOUT = float(os.environ.get("MICROMOUSE_STEP_TIMEOUT", "300.0"))


class Bridge(Node):
    def __init__(self):
        super().__init__("gazebo_sync_brain")
        # Deep command queue: we fire the whole path without waiting.
        self.pub = self.create_publisher(String, CMD_TOPIC, 256)
        # TRANSIENT_LOCAL so a late-joining listener (the GIF recorder) replays
        # the whole trail it missed -- the brain fires every cell in a sub-second
        # burst, faster than a fresh subscriber can finish discovery. Depth >
        # any path length keeps the full history.
        cell_qos = QoSProfile(
            depth=1024,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cell_pub = self.create_publisher(Point, CELL_TOPIC, cell_qos)
        self._lock = threading.Lock()
        self._sent = 0
        self._completed = 0
        self.create_subscription(Bool, DONE_TOPIC, self._on_done, 256)

    def _on_done(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._completed += 1

    def wait_for_subscribers(self, timeout: float = 10.0) -> bool:
        """Block until DDS has matched our command/cell subscribers.

        A brand-new publisher with the default RELIABLE+VOLATILE QoS drops any
        message it sends before discovery matches a subscriber. The controller
        and recorder are already running when the brain starts, so we just wait
        for the match before firing the whole path -- otherwise the first burst
        (resetToStart + every move) is silently lost and the robot never moves.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            cmd_subs = self.pub.get_subscription_count()
            cell_subs = self.cell_pub.get_subscription_count()
            if cmd_subs > 0 and cell_subs > 0:
                mms.log("[brain] subscribers matched (cmd=%d cell=%d)"
                        % (cmd_subs, cell_subs))
                return True
            time.sleep(0.05)
        mms.log("[brain] WARNING proceeding after %.0fs with cmd=%d cell=%d "
                "subscribers" % (timeout, self.pub.get_subscription_count(),
                                 self.cell_pub.get_subscription_count()))
        return False

    def publish_cell(self, col, row):
        self.cell_pub.publish(Point(x=float(col), y=float(row), z=0.0))

    def send(self, command: str):
        """Fire a command at the robot without waiting for it to finish."""
        with self._lock:
            self._sent += 1
        self.pub.publish(String(data=command))

    def wait_until_robot_done(self) -> bool:
        """Block until the robot has acknowledged every command we sent."""
        deadline = time.time() + STEP_TIMEOUT
        while time.time() < deadline:
            with self._lock:
                if self._completed >= self._sent:
                    return True
            time.sleep(0.02)
        with self._lock:
            mms.log("[brain] TIMEOUT robot did %d/%d moves"
                    % (self._completed, self._sent))
        return False


def _distance_text(value: int) -> str:
    return str(value) if value < FF.INF else "x"


def paint_distances(maze, dist):
    mms.clear_all_color()
    mms.clear_all_text()
    for x in range(maze.n):
        for y in range(maze.n):
            mms.set_text(x, y, _distance_text(dist[(x, y)]))
    for g in C.goal_cells(maze.n):
        mms.set_color(g[0], g[1], "g")


def run(bridge: Bridge):
    maze, data = spec_mod.read(SPEC_PATH)
    goals = set(tuple(g) for g in data["goal_cells"])
    dist = FF.flood_fill_distances(maze, list(goals))

    w, h = mms.maze_width(), mms.maze_height()
    if (w, h) != (maze.n, maze.n):
        mms.log("[brain] WARNING mms maze is {}x{} but spec is {}x{}".format(
            w, h, maze.n, maze.n))

    paint_distances(maze, dist)

    cur = tuple(data["start_cell"])
    heading = C.N
    # Wait for the controller (and recorder/brain_viz) to match our publishers
    # before firing the path, or DDS drops the first burst and the robot never
    # moves. See Bridge.wait_for_subscribers.
    bridge.wait_for_subscribers()
    # Send the robot back to the start cell first, so every run begins from the
    # same place even if the previous run left it at the center.
    bridge.send("resetToStart")
    mms.set_color(cur[0], cur[1], "b")
    bridge.publish_cell(*cur)
    mms.log("[brain] start", cur, "distance", dist[cur])

    while cur not in goals:
        if mms.was_reset():
            mms.log("[brain] reset pressed -- returning to start")
            mms.ack_reset()
            cur, heading = tuple(data["start_cell"]), C.N
            bridge.send("resetToStart")
            paint_distances(maze, dist)
            bridge.publish_cell(*cur)
            continue

        nxt_dir = P.pick_next_dir(maze, dist, cur, heading)
        if nxt_dir is None:
            mms.log("[brain] no improving neighbor at", cur, "-- stopping")
            break

        for t in P.turns_for(heading, nxt_dir):
            bridge.send(t)
            (mms.turn_right if t == "turnRight" else mms.turn_left)()
            heading = P.apply_turn(heading, t)

        bridge.send("moveForward")
        mms.move_forward()
        cur = maze.neighbor(cur[0], cur[1], nxt_dir)
        mms.set_color(cur[0], cur[1], "b")
        bridge.publish_cell(*cur)
        mms.log("[brain] now at", cur, "distance", dist[cur])

    if cur in goals:
        mms.log("[brain] mms reached the center; waiting for the robot...")
    # Keep the process (and mms run) alive until the robot drains its queue,
    # otherwise the last few commands would be lost when we shut down.
    bridge.wait_until_robot_done()
    mms.log("[brain] robot finished. Done.")


def main(argv=None):
    rclpy.init(args=argv)
    bridge = Bridge()
    spin = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin.start()
    try:
        run(bridge)
    except mms.MouseCrashedError:
        mms.log("[brain] mms reported a CRASH -- maze/spec mismatch?")
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
