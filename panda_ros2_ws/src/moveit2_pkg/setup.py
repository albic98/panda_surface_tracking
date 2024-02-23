import os
from setuptools import find_packages, setup


package_name = 'moveit2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(f'share/{package_name}/launch', ['launch/launch_moveit2.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pandaop',
    maintainer_email='pandaop@todo.todo',
    description='Move Franka Emika',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit2_arg_XYZ = moveit2_pkg.moveit2_arg_XYZ:main',
            'moveit2_insert_XYZ = moveit2_pkg.moveit2_insert_XYZ:main',
            'moveit2_servo = moveit2_pkg.moveit2_servo:main',
            'moveit2_HOME = moveit2_pkg.moveit2_HOME:main',
            'moveit2_JOINTS = moveit2_pkg.moveit2_JOINTS:main',        
            'moveit2_Publisher_Force_Position = moveit2_pkg.moveit2_Publisher_Force_Position:main',
            'moveit2_Force_multiple_points = moveit2_pkg.moveit2_Force_multiple_points:main', 
            'moveit2_Force_to_one_point = moveit2_pkg.moveit2_Force_to_one_point:main',
            'moveit2_interpolation = moveit2_pkg.moveit2_interpolation:main', 
            'moveit2_clicked_point = moveit2_pkg.moveit2_clicked_point:main',
            'moveit2_multiple_points = moveit2_pkg.moveit2_multiple_points:main',
            'SetCollisionBehaviour = moveit2_pkg.SetCollisionBehaviour:main',
      
        ],
    },
)
