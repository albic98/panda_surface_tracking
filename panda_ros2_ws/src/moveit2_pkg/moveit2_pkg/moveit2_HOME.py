import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 5)
        self.publisher = self.create_publisher(JointTrajectory, 'panda_arm_controller/joint_trajectory', 5)
        self.motion_completed = False

    def joint_state_callback(self, msg):
        if not self.motion_completed:
            self.motion_completed = True  # Set the flag to indicate that motion has completed

    def plan_and_execute_motion(self, joint_positions):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 5
        trajectory_msg.points.append(point)

        self.publisher.publish(trajectory_msg)
        self.get_logger().info("Pomak u HOME poziciju.")
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    # Prompt the user to trigger the motion to home position
    time.sleep(0.5)
    input("Stisni ENTER za pomak u HOME...")

    joint_positions_degrees = "0 -45 0 -135 0 92 45"
    joint_positions_degrees = [float(position) for position in joint_positions_degrees.split()]

    joint_positions_radians = [math.radians(position) for position in joint_positions_degrees]
    joint_state_subscriber.plan_and_execute_motion(joint_positions_radians)

    while rclpy.ok() and not joint_state_subscriber.motion_completed:
        rclpy.spin_once(joint_state_subscriber)  # Process a single iteration of the subscription callback
        time.sleep(0.8)
        break

    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
