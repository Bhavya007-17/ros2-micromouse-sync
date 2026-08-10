#!/usr/bin/env python3
"""Run the explorer end to end with no Gazebo and no mms GUI.

Three parts, one process plus one subprocess:

  * `raycast_lidar` publishes /scan from the maze truth file
  * a kinematic stand-in for the motion controller acks each command and
    reports where the robot now is
  * the real `explorer_brain`, run as a subprocess with an in-process mms host
    on its stdin/stdout (the same trick as run_automated_brain.py -- a shell
    pipe deadlocks, because the protocol is bidirectional)

Nothing here is a mock of the brain: it is the shipping brain, the shipping
sensor, and the shipping protocol. Only the robot's legs are simulated, which
is what lets this run in a second instead of a minute.

    python3 scripts/run_explorer_headless.py
"""
import argparse
import math
import os
import subprocess
import sys
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "micromouse")
sys.path.insert(0, PKG)

import rclpy                                              # noqa: E402
from rclpy.executors import MultiThreadedExecutor         # noqa: E402
from rclpy.node import Node                               # noqa: E402
from geometry_msgs.msg import Pose                        # noqa: E402
from std_msgs.msg import String, Bool                     # noqa: E402

from micromouse import spec as spec_mod                   # noqa: E402
from micromouse.raycast_lidar import RaycastLidar         # noqa: E402

BRAIN = os.path.join(PKG, "micromouse", "explorer_brain.py")


class KinematicController(Node):
    """Moves the robot on paper: exact cells, exact right angles.

    Stands in for cell_motion_controller so the protocol can be exercised
    without a simulator. It publishes the pose BEFORE acknowledging the
    command, so the scan the brain reads next is always taken from where the
    robot now is.
    """

    def __init__(self):
        super().__init__("kinematic_controller")
        data = spec_mod.read_public(spec_mod.public_path())
        self.cell = float(data["cell_size_m"])
        sx, sy = data["start_cell"]
        self.start = (sx * self.cell + self.cell / 2.0,
                      sy * self.cell + self.cell / 2.0,
                      math.pi / 2.0)
        self.x, self.y, self.yaw = self.start
        self.moves = 0

        self.pose_pub = self.create_publisher(Pose, "/maze/robot_pose", 10)
        self.done_pub = self.create_publisher(Bool, "/maze/step_complete", 16)
        self.create_subscription(String, "/maze/mms_command", self._on_cmd, 16)
        self.create_timer(0.05, self._publish_pose)

    def _publish_pose(self):
        msg = Pose()
        msg.position.x, msg.position.y = self.x, self.y
        msg.orientation.z = math.sin(self.yaw / 2.0)
        msg.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(msg)

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        if cmd == "resetToStart":
            self.x, self.y, self.yaw = self.start
        elif cmd == "turnLeft":
            self.yaw += math.pi / 2.0
        elif cmd == "turnRight":
            self.yaw -= math.pi / 2.0
        elif cmd == "moveForward":
            self.x += self.cell * math.cos(self.yaw)
            self.y += self.cell * math.sin(self.yaw)
            self.moves += 1
        self._publish_pose()
        self.done_pub.publish(Bool(data=True))


def report_gap(speed_steps):
    """Score the run against the real maze.

    Only the harness may do this -- it reads the truth file, which is exactly
    what the brain is forbidden. Returns None if the run never got that far.
    """
    from micromouse import flood_fill as FF, conventions as C
    from micromouse import run_metrics as RM

    maze, _ = spec_mod.read_truth(spec_mod.truth_path())
    best = FF.flood_fill_distances(maze, C.goal_cells(maze.n))[C.start_cell()]
    print(RM.describe_gap(speed_steps, best), flush=True)


def parse_speed_steps(text):
    for line in text.splitlines():
        if "speed run  :" in line:
            return int(line.split(":")[1].split()[0])
    return None


def run_brain(n, timeout=900):
    """Run the real brain and host the mms side of its protocol.

    stderr is captured rather than inherited so the run can be scored against
    the truth file afterwards; it is echoed unchanged so nothing is hidden.
    """
    import tempfile

    with tempfile.TemporaryFile("w+") as err:
        proc = subprocess.Popen([sys.executable, BRAIN],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=err, text=True, bufsize=1)
        try:
            serve_mms(proc, n)
            proc.wait(timeout=timeout)
        finally:
            if proc.poll() is None:
                proc.kill()
        err.seek(0)
        log = err.read()

    sys.stderr.write(log)
    steps = parse_speed_steps(log)
    if steps is not None:
        report_gap(steps)
    return proc.returncode or 0


def serve_mms(proc, n):
    """The mms side of the protocol: answer queries, ack motion, swallow the
    display commands that expect no reply."""
    silent = {"setColor", "clearColor", "clearAllColor", "setText",
              "clearText", "clearAllText", "setWall", "clearWall"}
    replies = {"mazeWidth": str(n), "mazeHeight": str(n),
               "wallFront": "false", "wallLeft": "false", "wallRight": "false",
               "moveForward": "ack", "turnLeft": "ack", "turnRight": "ack",
               "ackReset": "ack", "wasReset": "false"}
    for line in proc.stdout:
        parts = line.strip().split()
        if not parts or parts[0] in silent:
            continue
        reply = replies.get(parts[0])
        if reply is not None:
            proc.stdin.write(reply + "\n")
            proc.stdin.flush()


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--noise", type=float, default=0.0,
                   help="lidar noise stddev in metres")
    p.add_argument("--rays", type=int, default=72)
    p.add_argument("--attach", action="store_true",
                   help="a simulator and lidar are already running; just host "
                        "the mms protocol for the brain")
    args = p.parse_args(argv)

    data = spec_mod.read_public(spec_mod.public_path())
    n = int(data["n"])
    print("maze is %dx%d; the brain knows that, the start and the goal, and "
          "nothing else" % (n, n), flush=True)

    if args.attach:
        return run_brain(n)

    rclpy.init()
    lidar = RaycastLidar()
    lidar.set_parameters([
        rclpy.parameter.Parameter("noise_stddev",
                                  rclpy.Parameter.Type.DOUBLE, args.noise),
    ])
    lidar.noise = args.noise
    controller = KinematicController()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar)
    executor.add_node(controller)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    try:
        rc = run_brain(n)
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        lidar.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

    print("robot made %d cell moves in total" % controller.moves, flush=True)
    return proc.returncode or 0


if __name__ == "__main__":
    sys.exit(main())
