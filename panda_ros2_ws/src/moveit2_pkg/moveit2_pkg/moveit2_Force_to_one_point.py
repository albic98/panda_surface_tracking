#!/usr/bin/env python3

from threading import Thread, Event
import time
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from franka_msgs.msg import FrankaState
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


class PoseGoalNode(Node):
    def __init__(self):
        super().__init__('pose_goal')
        self.clicked_point_event = Event()
        self.move_to_home_flag = False


        # Create a single instance of MoveIt2 in the constructor
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
        )        

        # Subscribe to the topics
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.cartesian_subscription = self.create_subscription(Bool, '/cartesian_flag', self.cartesian_flag_callback, 10)
        self.franka_state_subscription = self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', self.check_force, 10)
        # Default values
        self.fixed_quaternion = [1.0, 0.0, 0.0, 0.0]
        self.cartesian_flag = True  # Set to True by default

    def clicked_point_callback(self, msg):
        self.position = [msg.point.x, msg.point.y, msg.point.z]

        # Print the clicked point
        self.get_logger().info(f"Koordinate kliknute točke: {self.position}")

        # Set the event to signal that a point has been clicked
        self.clicked_point_event.set()

    def check_force(self, msg):
        # Retrieve the latest force values from the robot state
        x_force = msg.o_f_ext_hat_k[0]
        y_force = msg.o_f_ext_hat_k[1]
        z_force = abs(msg.o_f_ext_hat_k[2])
        self.z_force = z_force

    def cartesian_flag_callback(self, msg):
        self.cartesian_flag = msg.data

    def move_to_pose(self, position, quat_xyzw, cartesian):
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        self.moveit2.wait_until_executed()

    def execute_all_moves(self):
        position = self.position
        quat_xyzw = self.fixed_quaternion
        cartesian = self.cartesian_flag
        # Move to the current point
        self.move_to_pose(position, quat_xyzw, cartesian)
        while not self.clicked_point_event.is_set():
            # Continuously check force while in motion
            z_force = self.z_force
            # Adjust z-coordinate based on force
            if 10 <= z_force <= 15:
                # Force is within the desired range
                time.sleep(0.05)
            elif self.z_force < 10:
                position[2] -= (10 - z_force) * 0.00035
                self.get_logger().info(f"Adjusting position: {position}")
                self.move_to_pose(position, quat_xyzw, cartesian)
            else:
                position[2] += (z_force - 15) * 0.00055
                self.get_logger().info(f"Adjusting position: {position}")
                self.move_to_pose(position, quat_xyzw, cartesian)

        # Reset the event for the next point
        self.clicked_point_event.clear()

def main():
    rclpy.init()

    node = PoseGoalNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    try:
        while rclpy.ok():
                node.clicked_point_event.wait()
                node.clicked_point_event.clear()
                node.execute_all_moves()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()