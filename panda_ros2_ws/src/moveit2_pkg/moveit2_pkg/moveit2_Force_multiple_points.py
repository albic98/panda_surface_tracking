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
        position = [msg.point.x, msg.point.y, msg.point.z - 0.0015]
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
            # self.get_logger().info("Svi punktovi za pomicanje:")
            # for point_idx, (position, _, cartesian_flag) in enumerate(self.points_to_move):
            #     self.get_logger().info(f"Punkt {point_idx + 1}: Pozicija: {position}")

            # Set the event to signal that interpolation is done
            self.clicked_point_event.set()

    def check_force(self, msg):
        # Retrieve the latest force values from the robot state
        x_force = msg.o_f_ext_hat_k[0]
        y_force = msg.o_f_ext_hat_k[1]
        z_force = - msg.o_f_ext_hat_k[2]
        self.z_force = z_force

    def cartesian_flag_callback(self, msg):
        self.cartesian_flag = msg.data
        

    def move_to_pose(self, position, quat_xyzw, cartesian):
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        self.moveit2.wait_until_executed()

    def interpolate_points(self, start_point, end_point):
        time.sleep(0.5)
        self.get_logger().info(f"Izvršavanje interpolacije između {self.iteration_number}. i {self.iteration_number+1}. točke.")
        self.iteration_number = self.iteration_number + 1
        interpolation_step = 0.005  # 5 mm
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

        # for j, (position, _, _) in enumerate(interpolated_all_points):
        #     self.get_logger().info(f"Interpolirana pozicija {j + 1}/{len(interpolated_all_points)}: {position}")
        # self.get_logger().info("Interpolacija završena uspješno.")
        time.sleep(0.5)

    
    def execute_all_moves(self):

        initial_point = self.points_to_move[0]
        initial_position, quat_xyzw, cartesian = initial_point
        elevated_position = [initial_position[0], initial_position[1], initial_position[2] + 0.015]

        # Move to the elevated position and wait for 0.5 seconds
        self.move_to_pose(elevated_position, quat_xyzw, cartesian)
        time.sleep(0.5)

        self.get_logger().info(f"Izvršavanje dolaska u sve {len(self.points_to_move)} pozicije.")
        for i, point in enumerate(self.points_to_move):
            position, quat_xyzw, cartesian = point
            self.get_logger().info(f"Gibanje u poziciju {i + 1}/{len(self.points_to_move)}: {position}")

            # Move to the current point
            self.move_to_pose(position, quat_xyzw, cartesian)
        
            while not self.clicked_point_event.is_set():

                # Continuously check force while in motion
                z_force = self.z_force
                self.get_logger().info(f"Z Sila tijekom gibanja: {z_force}")

                # Adjust z-coordinate based on force
                if 8 <= z_force <= 14:
                    # Force is within the desired range
                    break
                elif z_force < 0:
                        position[2] -= 0.004
                        # self.get_logger().info(f"Namještanje pozicije: {position}")
                        self.move_to_pose(position, quat_xyzw, cartesian)
                elif 0 < z_force < 8:
                    # Force is too low, go down on z-coordinate
                    if z_force >= 4:
                        position[2] -= (8 - z_force) * 0.00043
                        # self.get_logger().info(f"Namještanje pozicije: {position}")
                        self.move_to_pose(position, quat_xyzw, cartesian)
                        for next_point in self.points_to_move[i + 1:]:
                            next_point[0][2] = position[2] - 0.0004   
                    else:
                        position[2] -= 0.0002
                        # self.get_logger().info(f"Namještanje pozicije: {position}")
                        self.move_to_pose(position, quat_xyzw, cartesian)
                else:
                    # Force is too high, go up on z-coordinate
                    if z_force <= 20:
                        position[2] += (z_force - 14) * 0.00044
                        # self.get_logger().info(f"Namještanje pozicije: {position}")
                        self.move_to_pose(position, quat_xyzw, cartesian)
                        for next_point in self.points_to_move[i + 1:]:
                            next_point[0][2] = position[2] + 0.003
                    else:
                        position[2] += 0.0002
                        # self.get_logger().info(f"Namještanje pozicije: {position}")
                        self.move_to_pose(position, quat_xyzw, cartesian)

            # Reset the event for the next point
            self.clicked_point_event.clear()

        self.get_logger().info("Sva gibanja završena uspješno.")
        time.sleep(1.5)
        self.moveit2.move_to_pose(position=[0.4, 0.1, 0.15], quat_xyzw=self.fixed_quaternion, cartesian=self.cartesian_flag)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Povratak u početnu poziciju.")
        print("\n")
        new_number_of_points = int(input(f"Unesite novi broj točaka koje želite dostići: "))
        print("\n")
        self.number_of_points = new_number_of_points

def main():
    rclpy.init()

    node = PoseGoalNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    try:

        node.number_of_points = int(input("Broj točaka koje želite dostići: "))

        while rclpy.ok():
            # Wait for the user to click on a point
            node.clicked_point_event.wait()
            node.clicked_point_event.clear()

            # Add logic to determine when to execute all movements
            if len(node.points_to_move) >= node.number_of_points:

                node.execute_all_moves()
                node.points_to_move = []
                node.iteration_number = 1

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()