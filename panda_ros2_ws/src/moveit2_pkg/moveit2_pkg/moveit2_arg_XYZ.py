#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run moveit2_pkg move_to_point_XYZ_v2 --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[1.0, 0.0, 0.01, 0.0]" -p cartesian:=True`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


def main():
	rclpy.init()

	# Create node for this example
	node = Node("pose_goal")

	# Declare parameters for position and orientation
	node.declare_parameter("position", [0.5, -0.2, 0.5])
	node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.01, 0.0])
	node.declare_parameter("cartesian", False)

	# Create callback group that allows execution of callbacks in parallel without restrictions
	callback_group = ReentrantCallbackGroup()

	# Create MoveIt 2 interface
	moveit2 = MoveIt2(
		node=node,
		joint_names=panda.joint_names(),
		base_link_name=panda.base_link_name(),
		end_effector_name=panda.end_effector_name(),
		group_name=panda.MOVE_GROUP_ARM,
		callback_group=callback_group,
	)

	# Spin the node in background thread(s)
	executor = rclpy.executors.MultiThreadedExecutor(2)
	executor.add_node(node)
	executor_thread = Thread(target=executor.spin, daemon=True, args=())
	executor_thread.start()

	# Get parameters
	position = node.get_parameter("position").get_parameter_value().double_array_value
	position_list = node.get_parameter("position").get_parameter_value().double_array_value
	position[1] = -position[1]
	quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
	cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

	# Move to pose
	node.get_logger().info(f"Gibanje u {{poziciju: {list(position_list)}, orijentaciju: {list(quat_xyzw)}}}")

	moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
	moveit2.wait_until_executed()

	rclpy.shutdown()
	exit(0)


if __name__ == "__main__":
	main()
