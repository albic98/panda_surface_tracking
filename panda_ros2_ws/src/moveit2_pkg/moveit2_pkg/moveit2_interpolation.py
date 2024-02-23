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
        self.points_to_move = []  # List to store points
        self.number_of_points = None  # Number of points to reach
        self.iteration_number = 1 

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
        position = [msg.point.x, msg.point.y, msg.point.z-0.001]
        quat_xyzw = self.fixed_quaternion

        # Append the point to the list only if it's not already in the list
        if len(self.points_to_move) == 0 or position != self.points_to_move[-1][0]:
            self.points_to_move.append((position, quat_xyzw, self.cartesian_flag))

            # Print the clicked point
            self.get_logger().info(f"Koordinate kliknute točke: {position}")

        if len(self.points_to_move) == self.number_of_points:
            # Perform interpolation
            for i in range(len(self.points_to_move) - 1):
                start_point = self.points_to_move[i]
                end_point = self.points_to_move[i+1]
                self.interpolate_points(start_point, end_point)

            # Print all points in the terminal
            del self.points_to_move[:self.number_of_points]
            unique_points = []
            for point in self.points_to_move:
                if point not in unique_points:
                    unique_points.append(point)
            self.points_to_move = unique_points
            self.get_logger().info("Svi punktovi za pomicanje:")
            for point_idx, (position, _, cartesian_flag) in enumerate(self.points_to_move):
                self.get_logger().info(f"Punkt {point_idx + 1}: Pozicija: {position}")

            print("\n")
            new_number_of_points = int(input("Novi broj točaka koje želite dostići: "))
            print("\n\n")
            self.number_of_points = new_number_of_points
            # Set the event to signal that interpolation is done
            self.clicked_point_event.set()

    def check_force(self, msg):
        # Retrieve the latest force values from the robot state
        x_force = msg.o_f_ext_hat_k[0]
        y_force = msg.o_f_ext_hat_k[1]
        z_force = abs(msg.o_f_ext_hat_k[2])
        self.z_force = z_force

    def cartesian_flag_callback(self, msg):
        self.cartesian_flag = msg.data

    def interpolate_points(self, start_point, end_point):
        time.sleep(1.0)
        self.get_logger().info(f"Izvršavanje interpolacije između {self.iteration_number}. i {self.iteration_number+1}. točke.")
        self.iteration_number = self.iteration_number + 1
        interpolation_step = 0.005  # 5 mm in meters
        start_position, _, _ = start_point
        end_position, _, _ = end_point

        # Calculate the number of interpolated points needed for each axis
        num_interpolated_points_x = int(np.ceil(np.abs(end_position[0] - start_position[0]) / interpolation_step))
        num_interpolated_points_y = int(np.ceil(np.abs(end_position[1] - start_position[1]) / interpolation_step))
        num_interpolated_points_z = int(np.ceil(np.abs(end_position[2] - start_position[2]) / interpolation_step))

        # Choose the maximum number of points among axes
        num_interpolated_points = max(num_interpolated_points_x, num_interpolated_points_y, num_interpolated_points_z)

        interpolated_all_points = []

        for i in range(num_interpolated_points + 1):
            alpha = i / num_interpolated_points
            interpolated_position = [
                start + alpha * (end - start) for start, end in zip(start_position, end_position)
            ]
            interpolated_quat_xyzw = start_point[1]

            interpolated_all_points.append((interpolated_position, interpolated_quat_xyzw, start_point[2]))

        # Add all interpolated points to the points_to_move list
        self.points_to_move.extend(interpolated_all_points)

        for j, (position, _, _) in enumerate(interpolated_all_points):
            self.get_logger().info(f"Interpolirana pozicija {j + 1}/{len(interpolated_all_points)}: {position}")

        self.get_logger().info("Interpolacija završena uspješno.")
        time.sleep(1.0)

def main():
    rclpy.init()

    node = PoseGoalNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    try:
        # Ask for the number of points before starting to listen for clicked points
        node.number_of_points = int(input("Broj točaka koje želite dostići: "))

        while rclpy.ok():
            # Wait for the user to click on a point
            node.clicked_point_event.wait()
            node.clicked_point_event.clear()

            # Add logic to determine when to execute all movements
            if len(node.points_to_move) >= node.number_of_points:
                node.points_to_move = []
                node.iteration_number = 1

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()