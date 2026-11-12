#!/usr/bin/env python3
"""Ordinary ROS subscriber/controller; fails if the environment is not observable."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('developer_smoke', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
counts = dict(image=0, info=0, clock=0, joint=0, ack=0, tf=0)
buffer = Buffer()
listener = TransformListener(buffer, node)
stamp = None
state = None

def receive(key, msg):
    global stamp, state
    counts[key] += 1
    if key == 'image':
        stamp = msg.header.stamp
    if key == 'joint':
        state = list(msg.position)

subscriptions = []
for key, topic, cls in [('image', '/image_raw', Image), ('info', '/camera_info', CameraInfo),
                        ('clock', '/clock', Clock), ('joint', '/joint_states', JointState),
                        ('ack', '/p1sim/command_received', String)]:
    subscriptions.append(node.create_subscription(cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
publisher = node.create_publisher(Vector3, '/p1sim/gimbal_command', 10)
deadline = time.monotonic() + 25
next_command = 0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() > next_command:
        publisher.publish(Vector3(x=0.1, y=-0.05, z=0.))
        next_command = time.monotonic() + 1
    if stamp:
        try:
            buffer.lookup_transform('odom', 'camera_optical_frame', Time.from_msg(stamp))
            counts['tf'] += 1
        except Exception:
            pass
    if all(value >= 3 for value in counts.values()) and state and abs(state[0]-0.1) < 1e-5 and abs(state[1]+0.05) < 1e-5:
        break
passed = all(value >= 3 for value in counts.values()) and state and abs(state[0]-0.1) < 1e-5 and abs(state[1]+0.05) < 1e-5
print(json.dumps(dict(passed=bool(passed), received=counts, joint_position=state)))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
