import rclpy
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetFullCollisionBehavior, SetCartesianStiffness

DEFAULT_VALUES = {
    'torque_acc_lower': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
    'torque_acc_upper': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
    'torque_lower': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
    'torque_upper': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
    'force_acc_lower': [20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
    'force_acc_upper': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
    'force_lower': [20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
    'force_upper': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0],
    'cartesian_stiffness':[2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
}


class CollisionBehaviourInterface:
    def __init__(self):
        self.node = rclpy.create_node('collision_behaviour_interface')
        self.ft_collision_behaviour_handle = self.node.create_client(SetForceTorqueCollisionBehavior, '/param_service_server/set_force_torque_collision_behavior')
        self.full_collision_behaviour_handle = self.node.create_client(SetFullCollisionBehavior, '/param_service_server/set_full_collision_behavior')
        self.cartesian_stiffness_handle = self.node.create_client(SetCartesianStiffness, '/param_service_server/set_cartesian_stiffness')


        while not self.ft_collision_behaviour_handle.wait_for_service(timeout_sec=1.0) or not self.full_collision_behaviour_handle.wait_for_service(timeout_sec=1.0) or not self.cartesian_stiffness_handle.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for collision behaviour services...')

        self.node.get_logger().info('Collision behaviour services found.')

    def set_ft_contact_collision_behaviour(self, torque_lower=None, torque_upper=None, force_lower=None, force_upper=None, cartesian_stiffness=None):
        if torque_lower is None:
            torque_lower = DEFAULT_VALUES['torque_lower']
        if torque_upper is None:
            torque_upper = DEFAULT_VALUES['torque_upper']
        if force_lower is None:
            force_lower = DEFAULT_VALUES['force_lower']
        if force_upper is None:
            force_upper = DEFAULT_VALUES['force_upper']
        if cartesian_stiffness is None:
            cartesian_stiffness = DEFAULT_VALUES['cartesian_stiffness']

        request = SetForceTorqueCollisionBehavior.Request()
        request.lower_torque_thresholds_nominal = torque_lower
        request.upper_torque_thresholds_nominal = torque_upper
        request.lower_force_thresholds_nominal = force_lower
        request.upper_force_thresholds_nominal = force_upper

        future = self.ft_collision_behaviour_handle.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(f"CollisionBehaviourInterface: Collision behaviour change request: {'Success' if future.result().success else 'Failed!'}.\nDetails: {future.result().error}")
            return future.result().success
        else:
            self.node.get_logger().warn(f"CollisionBehaviourInterface: Collision behaviour change service call failed.")
            return False

    def set_force_threshold_for_contact(self, cartesian_force_values):
        return self.set_ft_contact_collision_behaviour(force_lower=cartesian_force_values)

    def set_force_threshold_for_collision(self, cartesian_force_values):
        return self.set_ft_contact_collision_behaviour(force_upper=cartesian_force_values)

    def set_collision_threshold(self, joint_torques=None, cartesian_forces=None):
        return self.set_ft_contact_collision_behaviour(torque_upper=joint_torques, force_upper=cartesian_forces)

    def set_contact_threshold(self, joint_torques=None, cartesian_forces=None):
        return self.set_ft_contact_collision_behaviour(torque_lower=joint_torques, force_lower=cartesian_forces)

    def set_cartesian_stiffness(self, stiffness_values):
        return self.set_ft_contact_collision_behaviour(cartesian_stiffness=stiffness_values)

def main(args=None):
    rclpy.init(args=args)
    collision_interface = CollisionBehaviourInterface()

    cartesian_stiffness_new = [2500.0, 2500.0, 2500.0, 100.0, 100.0, 100.0]
    joint_torques = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
    cartesian_forces = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0]

    collision_interface.set_collision_threshold(joint_torques=joint_torques, cartesian_forces=cartesian_forces)

    collision_interface.set_cartesian_stiffness(stiffness_values=cartesian_stiffness_new)

    rclpy.spin(collision_interface.node)
    collision_interface.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
