
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, Shutdown
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
	return LaunchDescription([
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([PathJoinSubstitution(
			[FindPackageShare('franka_moveit_config'), 'launch', 'moveit.launch.py'])]),
		),
		Node(
			package='moveit2_pkg',
			executable='SetCollisionBehaviour',
			name='SetCollisionBehaviour',
		),	
		Node(
			package='moveit2_pkg',
			executable='moveit2_Publisher_Force_Position',
			name='moveit2_Publisher_Force_Position',
		),	
		# Node(
		# 	package='moveit2_pkg',
		# 	executable='moveit2_Force_multiple_points',
		# 	name='moveit2_Force_multiple_points',
		# 	output='both',
		# 	arguments=[{'number_of_points': DeclareLaunchArgument('self.number_of_points')}]
		# ),
	])

