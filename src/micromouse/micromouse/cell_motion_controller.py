#!/usr/bin/env python3
"""Cell-motion controller -- one mms command, one cell (or one 90 deg turn).

Two backends, chosen with the `motion` parameter:

`velocity` (default) -- the real thing. Publishes /cmd_vel, reads back /odom,
    and closes the loop: turn until the heading error is inside tolerance, then
    drive until the wheels say a full cell pitch has passed. The robot's
    position is *earned* from odometry rather than asserted, so wheel slip
    shows up as a short move instead of being papered over. Needs the ros_gz
    bridge and a simulator actually stepping physics.

`pose` -- commands ABSOLUTE model poses through the Gazebo
    `/world/<world>/set_pose` service. Nothing to overshoot, nothing to drift,
    and no dependence on the /cmd_vel bridge, which is slow on a GPU-less WSL
    host. Every move is exact regardless of real-time factor, which makes it
    the reliable path for recording a demo -- but the robot is being placed,
    not driven, so it demonstrates nothing about localisation.

Decoupled design: the subscription callback only appends to an in-memory FIFO;
a worker thread executes one command at a time and publishes
/maze/step_complete after each. That makes it impossible to drop or reorder a
command. The omniscient baseline brain exploits this by firing its whole path at
once; the explorer brain deliberately does not, because it cannot know its next
move until this one has finished.

Subscribes /maze/mms_command ("moveForward"|"turnLeft"|"turnRight"|"resetToStart")
Publishes  /maze/step_complete (std_msgs/Bool) after each move finishes
           /maze/robot_pose    (geometry_msgs/Pose) -- what the LiDAR casts from
"""
import collections
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool

from micromouse import conventions as C
from micromouse import scan_to_walls as S


class CellMotionController(Node):
    def __init__(self):
        super().__init__("cell_motion_controller")
        self.declare_parameter("spec_path", "")
        self.declare_parameter("world_name", "micromouse")
        self.declare_parameter("model_name", "micromouse")
        self.declare_parameter("cell_size_m", 0.30)
        self.declare_parameter("robot_z", 0.04)
        self.declare_parameter("steps", 2)          # interpolation sub-steps
        self.declare_parameter("step_dt", 0.01)     # wall seconds per sub-step
        # "velocity" drives the wheels and reads /odom -- real motion, real
        # odometry, real chance of getting it wrong. "pose" teleports, which is
        # exact and needs no physics; keep it for recording on a host where the
        # simulator is too slow to be worth waiting for.
        self.declare_parameter("motion", "velocity")
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 3.0)
        self.declare_parameter("linear_gain", 4.0)
        self.declare_parameter("angular_gain", 4.0)
        self.declare_parameter("position_tolerance", 0.004)
        self.declare_parameter("angle_tolerance", 0.02)
        self.declare_parameter("move_timeout", 12.0)

        self.world = self.get_parameter("world_name").value
        self.model = self.get_parameter("model_name").value
        self.cell = float(self.get_parameter("cell_size_m").value)
        self.z = float(self.get_parameter("robot_z").value)
        self.steps = int(self.get_parameter("steps").value)
        self.step_dt = float(self.get_parameter("step_dt").value)
        self.motion = str(self.get_parameter("motion").value).lower()
        self.max_v = float(self.get_parameter("max_linear").value)
        self.max_w = float(self.get_parameter("max_angular").value)
        self.k_v = float(self.get_parameter("linear_gain").value)
        self.k_w = float(self.get_parameter("angular_gain").value)
        self.pos_tol = float(self.get_parameter("position_tolerance").value)
        self.ang_tol = float(self.get_parameter("angle_tolerance").value)
        self.move_timeout = float(self.get_parameter("move_timeout").value)

        sc = (0, 0)
        self.wall_thickness = 0.02
        spec_path = self.get_parameter("spec_path").value
        if spec_path:
            try:
                from micromouse import spec as spec_mod
                # Public spec only: the controller needs geometry, not walls.
                data = spec_mod.read_public(spec_path)
                self.cell = float(data["cell_size_m"])
                self.wall_thickness = float(data.get("wall_thickness_m", 0.02))
                sc = tuple(data["start_cell"])
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn("spec read failed (%s); using defaults"
                                       % e)

        # exact world pose we track (we know it precisely)
        self.start_x = sc[0] * self.cell + self.cell / 2.0
        self.start_y = sc[1] * self.cell + self.cell / 2.0
        self.start_yaw = math.pi / 2.0               # North
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw

        self.svc = "/world/%s/set_pose" % self.world
        self.done_pub = self.create_publisher(Bool, "/maze/step_complete", 10)
        # Match the brain's TRANSIENT_LOCAL trail QoS so all publishers of this
        # topic agree and late subscribers (the GIF recorder) get the start cell.
        cell_qos = QoSProfile(
            depth=1024,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cell_pub = self.create_publisher(Point, "/maze/current_cell",
                                              cell_qos)
        # The simulated LiDAR raycasts from whatever pose we report here, so
        # this has to be published continuously, not just on arrival.
        self.pose_pub = self.create_publisher(Pose, "/maze/robot_pose", 10)
        # Large depth: the brain dumps the whole path at once and the callback
        # only appends, so the middleware queue must hold a burst until drained.
        self.create_subscription(String, "/maze/mms_command", self._on_cmd, 256)

        # FIFO of pending commands + worker that plays them back into Gazebo.
        self._queue = collections.deque()
        self._cv = threading.Condition()
        self._snapped = False
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        if self.motion == "velocity":
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self._odom = None                     # (x, y, yaw), in world frame
            self._raw_odom = None                 # as the wheels report it
            self._odom_anchor = None              # odom frame -> world frame
            self._odom_cv = threading.Condition()
            self._scan = None
            # Where the grid says the robot should be. Absolute, so command
            # errors cannot accumulate -- see _execute_velocity.
            self.target_x, self.target_y = self.start_x, self.start_y
            self.target_yaw = self.start_yaw
            self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
            # Nothing to snap: the robot is spawned at the start cell and from
            # here on it has to earn its position by driving.
            self._snapped = True
            self.get_logger().info(
                "velocity controller ready (cell=%.3f m, max %.2f m/s) -- "
                "pose comes from /odom" % (self.cell, self.max_v))
        else:
            self._snap_attempts = 0
            self._init_timer = self.create_timer(6.0, self._snap_to_start)
            self.get_logger().info(
                "pose controller ready (cell=%.3f m, service=%s, steps=%d)"
                % (self.cell, self.svc, self.steps))

    # -- pose helpers --------------------------------------------------------
    def _world_to_cell(self):
        col = int(round((self.x - self.cell / 2.0) / self.cell))
        row = int(round((self.y - self.cell / 2.0) / self.cell))
        return col, row

    def _publish_cell(self):
        col, row = self._world_to_cell()
        self.cell_pub.publish(Point(x=float(col), y=float(row), z=0.0))

    def _snap_to_start(self):
        self._snap_attempts += 1
        if not self._set_pose(self.x, self.y, self.yaw):
            if self._snap_attempts < 10:
                self.get_logger().warn(
                    "set_pose failed (attempt %d) -- robot not spawned yet?"
                    % self._snap_attempts)
                return
            self.get_logger().error(
                "could not set_pose after %d attempts" % self._snap_attempts)
        self._init_timer.cancel()
        self._publish_cell()
        with self._cv:
            self._snapped = True
            self._cv.notify_all()
        self.get_logger().info("snapped robot to start cell %s"
                               % (self._world_to_cell(),))

    @staticmethod
    def _quat(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _publish_pose(self, x, y, yaw):
        qx, qy, qz, qw = self._quat(yaw)
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = x, y, self.z
        (msg.orientation.x, msg.orientation.y,
         msg.orientation.z, msg.orientation.w) = qx, qy, qz, qw
        self.pose_pub.publish(msg)

    def _set_pose(self, x, y, yaw):
        self._publish_pose(x, y, yaw)
        qx, qy, qz, qw = self._quat(yaw)
        req = ('name: "%s", position: {x: %.5f, y: %.5f, z: %.5f}, '
               'orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}'
               % (self.model, x, y, self.z, qx, qy, qz, qw))
        try:
            result = subprocess.run(
                ["gz", "service", "-s", self.svc,
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "3000", "--req", req],
                capture_output=True, text=True, timeout=5.0)
            if result.returncode != 0:
                err = (result.stderr or result.stdout or "").strip()
                self.get_logger().debug("set_pose rc=%d %s"
                                        % (result.returncode, err))
                return False
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("set_pose failed: %s" % e)
            return False

    def _interp_to(self, tx, ty, tyaw):
        x0, y0, yaw0 = self.x, self.y, self.yaw
        dyaw = C.normalize_angle(tyaw - yaw0)        # shortest arc
        n = max(1, self.steps)
        for i in range(1, n + 1):
            f = i / n
            self._set_pose(x0 + (tx - x0) * f,
                           y0 + (ty - y0) * f,
                           yaw0 + dyaw * f)
            time.sleep(self.step_dt)
        self.x, self.y, self.yaw = tx, ty, C.normalize_angle(tyaw)

    # -- velocity control ----------------------------------------------------
    def _anchor_odom(self, sample, world):
        """Pin the odom frame onto the maze.

        gz diff-drive reports odometry from wherever the robot spawned, with
        that spawn as the origin and yaw zero. The maze, the targets and the
        LiDAR all speak world coordinates. Without this the two frames differ by
        the start cell offset *and* a 90 degree rotation, and every range gets
        cast from the wrong place -- which shows up much later as a belief that
        has walled off the start cell.
        """
        self._odom_anchor = (sample, world)

    def _odom_to_world(self, sample):
        (ox, oy, oyaw), (wx, wy, wyaw) = self._odom_anchor
        dx, dy = sample[0] - ox, sample[1] - oy
        theta = wyaw - oyaw
        c, s = math.cos(theta), math.sin(theta)
        return (wx + c * dx - s * dy,
                wy + s * dx + c * dy,
                C.normalize_angle(wyaw + (sample[2] - oyaw)))

    def _on_odom(self, msg: Odometry):
        """The robot's only sense of where it is: integrated wheel rotation."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        sample = (p.x, p.y, yaw)
        if self._odom_anchor is None:
            self._anchor_odom(sample, (self.start_x, self.start_y,
                                       self.start_yaw))
        self._raw_odom = sample
        world = self._odom_to_world(sample)
        with self._odom_cv:
            self._odom = world
            self._odom_cv.notify_all()
        self.x, self.y, self.yaw = world
        self._publish_pose(*world)

    def _wait_for_odom(self, timeout=10.0):
        with self._odom_cv:
            if self._odom is None:
                self._odom_cv.wait(timeout=timeout)
            return self._odom

    def _pose_now(self):
        with self._odom_cv:
            return self._odom

    def _drive(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _halt(self):
        self._drive(0.0, 0.0)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _rotate_to(self, target_yaw) -> bool:
        """Turn in place until the heading error is inside tolerance."""
        deadline = time.time() + self.move_timeout
        while time.time() < deadline:
            pose = self._pose_now()
            if pose is None:
                break
            error = C.normalize_angle(target_yaw - pose[2])
            if abs(error) <= self.ang_tol:
                self._halt()
                return True
            self._drive(0.0, self._clamp(self.k_w * error, self.max_w))
            time.sleep(0.01)
        self._halt()
        self.get_logger().warn("rotate timed out")
        return False

    def _drive_to(self, tx, ty, target_yaw) -> bool:
        """Drive to an absolute point, holding an absolute heading.

        Progress is the *remaining distance along the heading*, not the distance
        already travelled: overshoot then reads as negative and the controller
        backs off instead of insisting it has further to go.
        """
        deadline = time.time() + self.move_timeout
        while time.time() < deadline:
            pose = self._pose_now()
            if pose is None:
                break
            remaining = ((tx - pose[0]) * math.cos(target_yaw)
                         + (ty - pose[1]) * math.sin(target_yaw))
            if abs(remaining) <= self.pos_tol:
                self._halt()
                return True
            heading_error = C.normalize_angle(target_yaw - pose[2])
            self._drive(self._clamp(self.k_v * remaining, self.max_v),
                        self._clamp(self.k_w * heading_error, self.max_w))
            time.sleep(0.01)
        self._halt()
        self.get_logger().warn("move timed out")
        return False

    def _on_scan(self, msg):
        self._scan = S.samples_from_ranges(msg.ranges, msg.angle_min,
                                           msg.angle_increment)

    def _sector_range(self, bearing):
        """Median range around a body-frame bearing, or None if that side is
        open (nothing near enough to measure against)."""
        samples = self._scan
        if not samples:
            return None
        window = sorted(
            r for a, r in samples
            if abs(C.normalize_angle(a - bearing)) <= S.DEFAULT_WINDOW_RAD
            and math.isfinite(r))
        if not window:
            return None
        median = window[len(window) // 2]
        return median if median < self.cell * S.DEFAULT_THRESHOLD_FRAC else None

    def _front_range(self):
        return self._sector_range(0.0)

    def _correct_lateral(self) -> bool:
        """Slide back onto the centre line of the corridor.

        Needed because holding the heading *by turning* curves the path: every
        mid-drive correction leaves a little sideways offset, and after a few
        cells the beam that should measure this cell's wall is measuring the
        next one's. With walls on both sides the offset is just half the
        difference between them.

        A differential drive cannot strafe, so this costs two turns. Only worth
        paying when the offset is real, hence the threshold.
        """
        left = self._sector_range(math.pi / 2.0)
        right = self._sector_range(-math.pi / 2.0)
        if left is None or right is None:
            return True                      # no reference on one side
        offset = (left - right) / 2.0        # positive: drifted to the right
        if abs(offset) <= 2 * self.pos_tol:
            return True
        pose = self._pose_now()
        if pose is None:
            return True
        sideways = C.normalize_angle(self.target_yaw + math.pi / 2.0)
        if not self._rotate_to(sideways):
            return False
        ok = self._drive_to(pose[0] + offset * math.cos(sideways),
                            pose[1] + offset * math.sin(sideways), sideways)
        return self._rotate_to(self.target_yaw) and ok

    def _recentre(self) -> bool:
        """Pull the robot back onto the grid using the walls it can see.

        Two corrections, both cheap and both classic micromouse:

          * heading -- turn back to the exact cardinal the grid expects, so a
            forward move that ended slightly skewed does not bias the next one
          * along the corridor -- if there is a wall ahead, close the gap until
            it sits at the distance a centred robot would measure

          * across the corridor -- slide back onto the centre line, see
            _correct_lateral
        """
        if not self._rotate_to(self.target_yaw):
            return False
        if not self._correct_lateral():
            return False
        ahead = self._front_range()
        if ahead is None:
            return True
        ideal = self.cell / 2.0 - self.wall_thickness / 2.0
        error = ahead - ideal
        if abs(error) <= self.pos_tol:
            return True
        pose = self._pose_now()
        if pose is None:
            return True
        # Nudge the target too, so the correction is not undone next move.
        self.target_x += error * math.cos(self.target_yaw)
        self.target_y += error * math.sin(self.target_yaw)
        return self._drive_to(self.target_x, self.target_y, self.target_yaw)

    def _execute_velocity(self, cmd: str) -> bool:
        """Drive the wheels toward the exact grid pose this command implies.

        The targets are absolute, recomputed from the ideal grid each time,
        never from where the robot currently thinks it is. That distinction is
        not cosmetic: chaining `turn to (current heading + 90 deg)` lets every
        turn keep its own error, and after a handful of moves the robot is
        pointing far enough off that the LiDAR reports a neighbouring cell's
        walls. Odometry still drifts against the world -- that is real and is
        what the LiDAR re-centering corrects -- but it no longer compounds one
        command into the next.
        """
        if self._wait_for_odom() is None:
            self.get_logger().error("no /odom -- is the ros_gz bridge running?")
            return False

        if cmd == "resetToStart":
            # Driving home blind is not the point of the exercise; put the robot
            # back the only way that is guaranteed, then carry on under wheels.
            self._halt()
            self.target_x, self.target_y = self.start_x, self.start_y
            self.target_yaw = self.start_yaw
            ok = self._set_pose(self.start_x, self.start_y, self.start_yaw)
            # The teleport moves the robot but not the wheels, so /odom does not
            # jump with it. Re-pin the frames or every later pose is off by the
            # distance we just skipped.
            time.sleep(0.2)
            if self._raw_odom is not None:
                self._anchor_odom(self._raw_odom,
                                  (self.start_x, self.start_y, self.start_yaw))
            return ok
        if cmd in ("turnLeft", "turnRight"):
            step = math.pi / 2.0 if cmd == "turnLeft" else -math.pi / 2.0
            self.target_yaw = C.normalize_angle(self.target_yaw + step)
            return self._rotate_to(self.target_yaw)
        if cmd == "moveForward":
            self.target_x += self.cell * math.cos(self.target_yaw)
            self.target_y += self.cell * math.sin(self.target_yaw)
            ok = self._drive_to(self.target_x, self.target_y, self.target_yaw)
            return self._recentre() and ok
        self.get_logger().warn("unknown command '%s'" % cmd)
        return False

    # -- command handling ----------------------------------------------------
    def _on_cmd(self, msg: String):
        # Fast path only: enqueue and return so the middleware queue never
        # overflows even when the brain fires the whole path at once.
        with self._cv:
            self._queue.append(msg.data.strip())
            self._cv.notify_all()

    def _execute(self, cmd: str) -> bool:
        if self.motion == "velocity":
            return self._execute_velocity(cmd)
        if cmd == "resetToStart":
            # Teleport straight back to the start cell so every run begins from
            # the same place regardless of where the last run ended.
            self.x, self.y, self.yaw = (self.start_x, self.start_y,
                                        self.start_yaw)
            self._set_pose(self.x, self.y, self.yaw)
        elif cmd == "turnLeft":
            self._interp_to(self.x, self.y, self.yaw + math.pi / 2.0)
        elif cmd == "turnRight":
            self._interp_to(self.x, self.y, self.yaw - math.pi / 2.0)
        elif cmd == "moveForward":
            self._interp_to(self.x + self.cell * math.cos(self.yaw),
                            self.y + self.cell * math.sin(self.yaw),
                            self.yaw)
        else:
            self.get_logger().warn("unknown command '%s'" % cmd)
            return False
        return True

    def _run_worker(self):
        # Wait until the robot has been snapped to the start cell.
        with self._cv:
            while not self._snapped and rclpy.ok():
                self._cv.wait(timeout=0.5)
        while rclpy.ok():
            with self._cv:
                while not self._queue and rclpy.ok():
                    self._cv.wait(timeout=0.5)
                if not rclpy.ok():
                    return
                cmd = self._queue.popleft()
            self._execute(cmd)
            # NOTE: we intentionally do NOT publish /maze/current_cell here.
            # The brain owns that topic so the trail window fills at mms speed
            # (instantly); republishing the robot's lagging cell would make the
            # trail rewind. Acknowledge every command (even unknown ones) so the
            # brain's end-of-run handshake counts stay aligned.
            self.done_pub.publish(Bool(data=True))


def main(argv=None):
    rclpy.init(args=argv)
    node = CellMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
