"""Compose unchanged ROS robot_state_publisher with the simulator's JointState."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot = '''<robot name="consumer"><link name="consumer_base"/>
<link name="consumer_yaw"/><link name="consumer_pitch"/>
<joint name="gimbal_yaw_joint" type="continuous"><parent link="consumer_base"/><child link="consumer_yaw"/><axis xyz="0 0 1"/></joint>
<joint name="gimbal_pitch_joint" type="continuous"><parent link="consumer_yaw"/><child link="consumer_pitch"/><axis xyz="0 1 0"/></joint></robot>'''
    return LaunchDescription([Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='developer_vehicle_consumer', parameters=[{'robot_description': robot, 'use_sim_time': True}])])
