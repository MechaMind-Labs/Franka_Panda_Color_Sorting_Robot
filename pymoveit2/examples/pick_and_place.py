#!/usr/bin/env python3
"""
Move the robot to the color object detected by ColorDetector.
Listens to /color_coordinates and only moves when the color matches the target param.
"""

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


class PoseMover(Node):
    def __init__(self):
        super().__init__("pose_mover")

        # Declare param for target color
        self.declare_parameter("target_color", "R")   # default Red
        self.target_color = self.get_parameter("target_color").value.upper()

        # To prevent moving multiple times
        self.already_moved = False

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Subscriber to color coordinates
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )

        self.get_logger().info(
            f"PoseMover started. Waiting for {self.target_color} from /color_coordinates..."
        )

    def coords_callback(self, msg):
        if self.already_moved:
            return  # ignore further detections

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                x, y, z = float(x), float(y), float(z)

                self.get_logger().info(
                    f"Target {self.target_color} detected at: [{x:.3f}, {y:.3f}, {z:.3f}]"
                )

                # Orientation (gripper facing down)
                quat_xyzw = [0.0, 1.0, 0.0, 0.0]
                position = [x, y, z-0.85]

                # Move robot
                self.get_logger().info(
                    f"Moving to position: {position}, orientation: {quat_xyzw}"
                )
                self.moveit2.move_to_pose(
                    position=position, quat_xyzw=quat_xyzw, cartesian=False
                )
                self.moveit2.wait_until_executed()

                self.get_logger().info(f"Reached {self.target_color} object.")
                self.already_moved = True
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")


def main():
    rclpy.init()
    node = PoseMover()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
