#!/usr/bin/env python3

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

	# Get position from user input
	position_str = input("Enter position as a list [x, y, z]: ")
	position_list = list(map(float, position_str.strip('[]').split(',')))
	position = list(map(float, position_str.strip('[]').split(',')))
	position[1] = -position[1]

	# Get quaternion from user input
	# quat_str = input("Enter quaternion as a list [x, y, z, w]: ")
	# quat_xyzw = list(map(float, quat_str.strip('[]').split(',')))
	quat_xyzw = [1.0, 0.0, 0.0, 0.0]

	# Get cartesian flag from user input
	# cartesian_str = input("Enter True for cartesian, False for joint space: ")
	# cartesian = bool(cartesian_str.lower() == 'true')
	cartesian = True

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

	# Move to pose
	node.get_logger().info(
	f"Moving to {{position: {list(position_list)}, quat_xyzw: {list(quat_xyzw)}}}"
	)
	moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
	moveit2.wait_until_executed()
	print("Movement finished.")
	
	rclpy.shutdown()
	exit(0)

if __name__ == "__main__":
	main()
    
    
