import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateSubscriber(Node):
	def __init__(self):
		super().__init__('joint_state_subscriber')
		self.subscription = self.create_subscription(JointState,'joint_states',self.joint_state_callback,10)

		self.publisher = self.create_publisher(JointTrajectory,'panda_arm_controller/joint_trajectory',10)
        
         
	def joint_state_callback(self, msg):
		joint_positions = msg.position
		joint_names = msg.name
		print("Joint Positions:")
		for name, position in zip(joint_names, joint_positions):
			if "panda_finger" not in name:
				position_degrees = math.degrees(position)
				print(f"{name} (stupnjevi): {position_degrees:.2f} °")
				# print(f"{name} (radians): {position:.2f} radians")
			print("--------------------------------------")
        

	def plan_and_execute_motion(self, joint_positions):
		trajectory_msg = JointTrajectory()
		trajectory_msg.header.stamp = self.get_clock().now().to_msg()
		trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4','panda_joint5', 'panda_joint6', 'panda_joint7']
				              
		point = JointTrajectoryPoint()
		point.positions = joint_positions
		point.time_from_start.sec = 4
		trajectory_msg.points.append(point)

		self.publisher.publish(trajectory_msg)
		print("Pokret isplaniran i odrađen.")
        
        
def main(args=None):
	rclpy.init(args=args)
	joint_state_subscriber = JointStateSubscriber()

	rclpy.spin_once(joint_state_subscriber)

	try:
		while True:
			joint_positions_degrees = input("Unesi kuteve zakreta zglobova (u °, s razmakom): ")
			joint_positions_degrees = [float(position) for position in joint_positions_degrees.split()]

			joint_positions_radians = [math.radians(position) for position in joint_positions_degrees]

			joint_state_subscriber.plan_and_execute_motion(joint_positions_radians)
			time.sleep(2)  # Delay for 2 seconds to allow execution

			rclpy.spin_once(joint_state_subscriber)

	except KeyboardInterrupt:
		print(" Korisnik zaustavio program.")
		pass

	joint_state_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
