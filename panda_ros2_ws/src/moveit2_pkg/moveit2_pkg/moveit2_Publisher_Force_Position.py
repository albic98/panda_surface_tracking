#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped, Wrench, Vector3
from franka_msgs.msg import FrankaState
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__('robot_state_subscriber')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the topics
        self.franka_state_subscription = self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', self.robot_state_callback, 10)
        self.franka_force_publisher = self.create_publisher(WrenchStamped,'/force_topic', 10)
        self.franka_pose_publisher = self.create_publisher(PoseStamped,'/position_topic', 10)
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        # self.timer_2 = self.create_timer(0.2, self.publish_force)

        # Get current time
        self.current_time = self.get_clock().now()

    def robot_state_callback(self, msg):
        x_force = msg.o_f_ext_hat_k[0]
        y_force = msg.o_f_ext_hat_k[1]
        z_force = abs(msg.o_f_ext_hat_k[2])

        # Create a WrenchStamped message and fill in the values
        wrench_msg = WrenchStamped(
            header=Header(
                stamp=self.current_time.to_msg(),
                frame_id='panda_link0'
            ),
            wrench=Wrench(
                force=Vector3(
                    # x=x_force,
                    # y=y_force,
                    z = z_force
                ),
                torque=Vector3(
                    #x=x_torque,
                    #y=y_torque,
                    #z=z_torque
                )
            )
        )
        # Publish the WrenchStamped message
        self.franka_force_publisher.publish(wrench_msg)
        time.sleep(0.1)

    def publish_pose(self):

        try:
            # Get the transform from panda_link0 to panda_ticalo_tcp
            transform = self.tf_buffer.lookup_transform('panda_link0', 'panda_ticalo_tcp', rclpy.time.Time())

            # Extract translation values
            translation = transform.transform.translation

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.current_time.to_msg()
            pose_msg.header.frame_id = 'panda_link0'
            pose_msg.pose.position.x = translation.x
            pose_msg.pose.position.y = translation.y
            pose_msg.pose.position.z = translation.z

            #Publish Pose Stamped message
            self.franka_pose_publisher.publish(pose_msg)
            # self.get_logger().info('Transformation sucessful.')
        except TransformException as ex:
            self.get_logger().info(f'Could not transform panda_link0 to panda_ticalo_tcp: {ex}')
            return
 

def main(args=None):
    rclpy.init(args=args)

    subscriber_node = RobotStateSubscriber()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass

    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

