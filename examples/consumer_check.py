"""Verify the unmodified robot_state_publisher actually consumes joint motion."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init()
node = Node('consumer_transform_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + 12
passed = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        value = buffer.lookup_transform('consumer_base', 'consumer_pitch', Time())
        q = value.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pitch = math.asin(2*(q.w*q.y-q.z*q.x))
        if abs(yaw-0.1) < 1e-5 and abs(pitch+0.05) < 1e-5:
            passed = True
            break
    except Exception:
        pass
print(json.dumps({'passed': passed, 'consumer': 'robot_state_publisher', 'yaw': yaw if passed else None, 'pitch': pitch if passed else None}))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
