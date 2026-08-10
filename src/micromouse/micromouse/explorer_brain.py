#!/usr/bin/env python3
"""The blind brain -- solves a maze it has never seen.

Compare `gazebo_sync_brain.py`, which reads every wall from the truth file and
fires the whole solution in one burst. This one starts knowing only the grid
size, the start cell and where the goal is. Every wall it knows, it learned from
`/scan`.

That inverts the old architecture. The baseline brain could run ahead of the
robot because the answer was already on disk; this one cannot know move n+1
until the robot has physically arrived at cell n and taken a reading there. So
it runs in strict lockstep:

    send one command -> wait for /maze/step_complete -> read /scan -> learn
    -> re-flood -> decide

mms is the belief display. Discovered walls are drawn with setWall, and the
distance field painted into the cells is the robot's own, computed from what it
has sensed -- so the mms window shows the map filling in and the numbers
settling, while Gazebo shows the maze that was there all along.

Reads the PUBLIC spec only. `spec.read_public` refuses a file containing walls,
so pointing this at the truth file fails loudly instead of quietly cheating.

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
from sensor_msgs.msg import LaserScan

from micromouse import conventions as C
from micromouse import explorer as E
from micromouse import flood_fill as FF
from micromouse import mms_interface as mms
from micromouse import planner as P
from micromouse import run_metrics as RM
from micromouse import scan_to_walls as S
from micromouse import spec as spec_mod
from micromouse import belief_maze as B

CMD_TOPIC = os.environ.get("MICROMOUSE_CMD_TOPIC", "/maze/mms_command")
DONE_TOPIC = os.environ.get("MICROMOUSE_DONE_TOPIC", "/maze/step_complete")
CELL_TOPIC = os.environ.get("MICROMOUSE_CELL_TOPIC", "/maze/current_cell")
BELIEF_TOPIC = os.environ.get("MICROMOUSE_BELIEF_TOPIC", "/maze/belief")

# How long to wait for the robot to finish one command before giving up.
STEP_TIMEOUT = float(os.environ.get("MICROMOUSE_STEP_TIMEOUT", "60.0"))
# A scan must be newer than the arrival it describes; this is how long we allow
# for one to show up.
SCAN_TIMEOUT = float(os.environ.get("MICROMOUSE_SCAN_TIMEOUT", "5.0"))

PHASE_COLOR = {E.SEARCH: "b", E.RETURN: "y", E.SPEED: "g"}
WALL_LETTER = {C.N: "n", C.E: "e", C.S: "s", C.W: "w"}


class Bridge(Node):
    """Motion commands out, arrival acks and laser scans in."""

    def __init__(self):
        super().__init__("explorer_brain")
        self.pub = self.create_publisher(String, CMD_TOPIC, 16)
        trail_qos = QoSProfile(
            depth=1024,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cell_pub = self.create_publisher(Point, CELL_TOPIC, trail_qos)
        self.belief_pub = self.create_publisher(String, BELIEF_TOPIC, trail_qos)

        self._lock = threading.Lock()
        self._completed = 0
        self._scan = None
        self._scan_seq = 0
        self.create_subscription(Bool, DONE_TOPIC, self._on_done, 16)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    # -- inbound -------------------------------------------------------------
    def _on_done(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._completed += 1

    def _on_scan(self, msg: LaserScan):
        samples = S.samples_from_ranges(msg.ranges, msg.angle_min,
                                        msg.angle_increment)
        with self._lock:
            self._scan = samples
            self._scan_seq += 1

    # -- outbound ------------------------------------------------------------
    def wait_for_subscribers(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        mms.log("[explorer] WARNING no controller listening on %s" % CMD_TOPIC)
        return False

    def send_and_wait(self, command: str) -> bool:
        """Fire one command and block until the robot says it finished.

        The lockstep the whole design turns on: no sensing is valid until the
        robot has actually arrived.
        """
        with self._lock:
            target = self._completed + 1
        self.pub.publish(String(data=command))
        deadline = time.time() + STEP_TIMEOUT
        while time.time() < deadline:
            with self._lock:
                if self._completed >= target:
                    return True
            time.sleep(0.005)
        mms.log("[explorer] TIMEOUT waiting for '%s'" % command)
        return False

    def settled_reading(self, cell_size, agreements=2,
                        timeout: float = SCAN_TIMEOUT):
        """Wait until consecutive scans agree on the walls, and return those.

        Waiting for merely the *next* scan is not enough, and this is not
        theoretical -- it corrupts the map. The lidar publishes on its own
        clock, so a scan can be sent after the robot has arrived but computed
        from the pose it had a moment earlier. The brain then records the
        previous cell's walls against the current cell, and the belief is
        quietly wrong from then on.

        The robot is stationary while we ask, so once the pose has propagated
        every scan reads the same. Requiring consecutive agreement therefore
        rejects the in-flight stale one without needing the sensor to tell us
        which pose it used -- which matters because the real Gazebo lidar
        cannot.
        """
        deadline = time.time() + timeout
        with self._lock:
            seen = self._scan_seq
        last, streak = None, 0
        while time.time() < deadline:
            with self._lock:
                samples = self._scan if self._scan_seq > seen else None
                seen = max(seen, self._scan_seq)
            if samples is None:
                time.sleep(0.002)
                continue
            reading = S.walls_from_scan(samples, cell_size)
            if reading == last:
                streak += 1
                if streak >= agreements:
                    return reading
            else:
                last, streak = reading, 1
        if last is not None:
            mms.log("[explorer] WARNING scans never settled; using the last one")
        return last

    def publish_cell(self, col, row):
        self.cell_pub.publish(Point(x=float(col), y=float(row), z=0.0))

    def publish_belief(self, payload: str):
        self.belief_pub.publish(String(data=payload))


def paint(belief, dist, cell, phase, drawn):
    """Mirror the robot's belief into mms.

    Only newly learned walls are drawn -- redrawing all 256 cells every step
    would swamp the protocol channel.
    """
    for (x, y, d) in belief_walls(belief):
        if (x, y, d) in drawn:
            continue
        drawn.add((x, y, d))
        mms.set_wall(x, y, WALL_LETTER[d])

    for x in range(belief.n):
        for y in range(belief.n):
            v = dist[(x, y)]
            mms.set_text(x, y, str(v) if v < FF.INF else "x")
    mms.set_color(cell[0], cell[1], PHASE_COLOR.get(phase, "b"))


def belief_walls(belief):
    """Every confirmed wall, each physical wall reported once."""
    for x in range(belief.n):
        for y in range(belief.n):
            for d in (C.N, C.E):
                if belief.state(x, y, d) == B.WALL:
                    yield (x, y, d)
            if y == 0 and belief.state(x, y, C.S) == B.WALL:
                yield (x, y, C.S)
            if x == 0 and belief.state(x, y, C.W) == B.WALL:
                yield (x, y, C.W)


def encode_belief(belief, phase, cell):
    """Compact wire format for brain_viz: one char per wall side per cell."""
    rows = []
    for y in range(belief.n):
        row = []
        for x in range(belief.n):
            row.append("".join(
                "?" if belief.state(x, y, d) == B.UNKNOWN
                else ("#" if belief.state(x, y, d) == B.WALL else ".")
                for d in (C.N, C.E, C.S, C.W)))
        rows.append(",".join(row))
    return "%s|%d,%d|%s" % (phase, cell[0], cell[1], ";".join(rows))


def run(bridge: Bridge):
    data = spec_mod.read_public(spec_mod.public_path())
    n = int(data["n"])
    start = tuple(data["start_cell"])
    goals = [tuple(g) for g in data["goal_cells"]]
    cell_size = float(data["cell_size_m"])

    w, h = mms.maze_width(), mms.maze_height()
    if (w, h) != (n, n):
        mms.log("[explorer] WARNING mms maze is %dx%d but spec says %dx%d"
                % (w, h, n, n))

    ex = E.Explorer(n, start, goals)
    metrics = RM.RunMetrics()
    drawn = set()

    mms.clear_all_color()
    mms.clear_all_text()
    for g in goals:
        mms.set_color(g[0], g[1], "g")

    bridge.wait_for_subscribers()
    bridge.send_and_wait("resetToStart")

    cell, heading = start, C.N
    bridge.publish_cell(*cell)
    mms.log("[explorer] start at %s -- nothing known yet" % (cell,))

    while True:
        relative = bridge.settled_reading(cell_size)
        if relative is None:
            mms.log("[explorer] no scan at %s -- is the lidar running?" % (cell,))
            break
        ex.observe(cell, heading, relative)

        direction = ex.step(cell, heading)
        phase = ex.phase
        dist = ex.distances()
        paint(ex.belief, dist, cell, phase, drawn)
        bridge.publish_belief(encode_belief(ex.belief, phase, cell))
        metrics.note(phase, cell, ex.belief)

        if phase == E.DONE:
            break
        if direction is None:
            mms.log("[explorer] stuck at %s facing %s in phase %s: "
                    "dist=%s open=%s walls=%s"
                    % (cell, heading, phase, dist[cell],
                       [d for d in (C.N, C.E, C.S, C.W)
                        if ex.belief.optimistic_view().open_between(
                            cell[0], cell[1], d)],
                       [(d, ex.belief.state(cell[0], cell[1], d))
                        for d in (C.N, C.E, C.S, C.W)]))
            break

        for turn in P.turns_for(heading, direction):
            if not bridge.send_and_wait(turn):
                return metrics, ex
            (mms.turn_right if turn == "turnRight" else mms.turn_left)()
            heading = P.apply_turn(heading, turn)

        if not bridge.send_and_wait("moveForward"):
            return metrics, ex
        mms.move_forward()
        metrics.record_move(phase)
        cell = ex.belief.neighbor(cell[0], cell[1], direction)
        bridge.publish_cell(*cell)

    mms.log(metrics.summary(ex))
    return metrics, ex


def main(argv=None):
    rclpy.init(args=argv)
    bridge = Bridge()
    spin = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin.start()
    try:
        run(bridge)
    except mms.MouseCrashedError:
        mms.log("[explorer] mms reported a CRASH -- the robot tried to move "
                "through a wall it believed was open")
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
