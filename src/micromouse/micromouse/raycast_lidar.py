#!/usr/bin/env python3
"""Simulated LiDAR: publishes a real LaserScan by ray-marching the maze.

This node owns the ground truth (`maze_truth.json`) and hands out nothing but
ranges. The explorer brain subscribes to `/scan` and has no other way to learn
where the walls are, so the separation is enforced by process boundaries rather
than by discipline.

Why this exists alongside the real Gazebo `gpu_lidar`: `gpu_lidar` needs the
Ogre render engine, which does not load on a GPU-less WSL host (see
docs/DESIGN.md). This path needs no rendering at all, so the demo always runs.
Switch with `lidar:=gz` when a GPU is available -- the brain cannot tell the
difference, because both publish the same message on the same topic.

Publishes  /scan              (sensor_msgs/LaserScan)
Subscribes /maze/robot_pose   (geometry_msgs/Pose)
"""
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

from micromouse import raycast as R
from micromouse import spec as spec_mod


def yaw_from_quaternion(q) -> float:
    """Yaw only -- the robot is planar, so roll and pitch are noise."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RaycastLidar(Node):
    def __init__(self):
        super().__init__("raycast_lidar")
        self.declare_parameter("truth_path", "")
        self.declare_parameter("num_rays", 72)
        self.declare_parameter("range_max", 4.0)
        self.declare_parameter("range_min", 0.0)
        self.declare_parameter("noise_stddev", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("pose_topic", "/maze/robot_pose")
        self.declare_parameter("seed", 0)

        path = (self.get_parameter("truth_path").value
                or spec_mod.truth_path())
        # Deliberately not caught: without ground truth this node has nothing to
        # simulate, and failing here is far better than publishing empty scans
        # that the brain would read as open space.
        self.maze, data = spec_mod.read_truth(path)
        self.cell = float(data["cell_size_m"])
        self.wall_thickness = float(data.get("wall_thickness_m", 0.0))

        self.num_rays = int(self.get_parameter("num_rays").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.noise = float(self.get_parameter("noise_stddev").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.rng = random.Random(int(self.get_parameter("seed").value))

        self.angles = R.fan_angles(self.num_rays)
        self.angle_min = self.angles[0]
        self.angle_increment = (2.0 * math.pi / self.num_rays)

        # Start at the middle of the start cell so a scan is available even
        # before the controller has said anything.
        sx, sy = data["start_cell"]
        self.x, self.y = (sx * self.cell + self.cell / 2.0,
                          sy * self.cell + self.cell / 2.0)
        self.yaw = math.pi / 2.0

        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.create_subscription(Pose,
                                 self.get_parameter("pose_topic").value,
                                 self._on_pose, 10)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "raycast lidar ready (%d rays, range %.1f m, noise %.3f m, maze %s)"
            % (self.num_rays, self.range_max, self.noise, path))

    def _on_pose(self, msg: Pose):
        self.x, self.y = msg.position.x, msg.position.y
        self.yaw = yaw_from_quaternion(msg.orientation)

    def _publish(self):
        ranges = R.scan(self.maze, self.cell, self.x, self.y, self.yaw,
                        self.angles, self.range_max,
                        wall_thickness=self.wall_thickness,
                        noise_stddev=self.noise, rng=self.rng)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_min
                              + (self.num_rays - 1) * self.angle_increment)
        msg.angle_increment = float(self.angle_increment)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [float(r) for r in ranges]
        self.pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = RaycastLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
