#!/usr/bin/env python3

from threading import Thread, Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
import time

class PoseGoalNode(Node):
	def __init__(self):
		super().__init__('pose_goal')
		self.clicked_point_event = Event()
		self.points_to_move = []  # List to store points
		self.number_of_points = None  # Number of points to reach

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

		# Default values
		self.fixed_quaternion = [1.0, 0.0, 0.0, 0.0]
		self.cartesian_flag = True # Set to True by default

	def clicked_point_callback(self, msg):
		position = [msg.point.x, msg.point.y, msg.point.z]
		quat_xyzw = self.fixed_quaternion

		# Append the point to the list
		self.points_to_move.append((position, quat_xyzw, self.cartesian_flag))

		# Print the clicked point
		self.get_logger().info(f"Koordinate kliknute točke: {position}")

		# Set the event to signal that a point has been clicked
		self.clicked_point_event.set()

	def cartesian_flag_callback(self, msg):
		self.cartesian_flag = msg.data

	def move_to_pose(self, position, quat_xyzw, cartesian):
		self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
		self.moveit2.wait_until_executed()
		self.get_logger().info("Gibanje završeno uspješno.")

	def execute_all_moves(self):
		time.sleep(1.0)
		self.get_logger().info(f"Izvršavanje dolaska u sve {len(self.points_to_move)} pozicije.")
		for i, point in enumerate(self.points_to_move):
			position, quat_xyzw, cartesian = point
			self.get_logger().info(f"Gibanje u poziciju {i + 1}/{len(self.points_to_move)}: {position}")
			self.move_to_pose(position, quat_xyzw, cartesian)
			# time.sleep(0.2)
                
		self.get_logger().info("Sva gibanja završena uspješno.")
		time.sleep(1.0)
		self.moveit2.move_to_pose(position=[0.35, 0.0, 0.3], quat_xyzw=self.fixed_quaternion, cartesian=self.cartesian_flag)
		self.moveit2.wait_until_executed()
		self.get_logger().info("Povratak u HOME poziciju.")
		print("")
		new_number_of_points = int(input(f"Unesite novi broj točaka koje želite dostići: "))
		self.number_of_points = new_number_of_points

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
				node.execute_all_moves()
				node.points_to_move = []

	except KeyboardInterrupt:
		pass
	finally:
		rclpy.shutdown()

if __name__ == "__main__":
	main()
