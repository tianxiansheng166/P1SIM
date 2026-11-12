#!/usr/bin/env python3
"""Observe a running AutoAim loop without publishing algorithm inputs or commands."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data as sensor_qos
from rclpy.time import Time
from rm_interfaces.msg import Armors, Target, GimbalCmd
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--timeout', type=float, default=60)
parser.add_argument('--output', default='/data/autoaim-check.json')
parser.add_argument('--command-topic', default='/armor_planner/cmd_gimbal')
parser.add_argument('--tracker-topic', default='/tracker/target')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('autoaim_loop_observer', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', value=True)])
buffer = Buffer()
listener = TransformListener(buffer, node)
proof = {}
commands = deque(maxlen=10000)
receipts = deque(maxlen=10000)
applied = deque(maxlen=1000)
last_info = None


def key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.header.frame_id, msg.yaw, msg.pitch, bool(msg.fire))


def armors(msg):
    if msg.armors:
        proof.setdefault('nonempty_armors', dict(count=len(msg.armors), stamp_sec=msg.header.stamp.sec))


def target(msg):
    if msg.tracking:
        proof.setdefault('tracking', dict(id=msg.id, stamp_sec=msg.header.stamp.sec))


def command(msg):
    if msg.control:
        commands.append(key(msg))
        proof.setdefault('control', dict(yaw=msg.yaw, pitch=msg.pitch, fire=bool(msg.fire)))


def receipt(msg):
    data = json.loads(msg.data)
    if data.get('control'):
        receipts.append(data)


def applied_pose(msg):
    data = json.loads(msg.data)
    if max(abs(data['yaw_rad']), abs(data['pitch_rad'])) > 1e-5:
        applied.append(data)


def info(msg):
    global last_info
    last_info = msg.header.stamp


def image(msg):
    if last_info is None or msg.header.stamp.sec <= 0:
        return
    # CameraInfo may arrive immediately after its Image. Accept the preceding
    # calibrated sample, while independently checking this exact Image's TF.
    try:
        buffer.lookup_transform('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        proof['source_time_tf'] = dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec,
                                      frame=msg.header.frame_id, camera_info_sec=last_info.sec)
    except TransformException:
        pass


def joint(msg):
    joints = dict(zip(msg.name, msg.position))
    if not all(k in joints for k in ('gimbal_yaw_joint', 'gimbal_pitch_joint')):
        return
    for pose in reversed(applied):
        if Time.from_msg(msg.header.stamp).nanoseconds < pose['simulation_time_ns']:
            continue
        if max(abs(joints['gimbal_yaw_joint']-pose['yaw_rad']),
               abs(joints['gimbal_pitch_joint']-pose['pitch_rad'])) > 1e-7:
            continue
        for data in reversed(receipts):
            identity = (data['stamp_sec'], data['stamp_nanosec'], data['frame_id'],
                        data['yaw_rad'], data['pitch_rad'], data['fire_requested'])
            if identity not in commands or data['fire_supported']:
                continue
            if max(abs(data['yaw_rad']-pose['yaw_rad']), abs(data['pitch_rad']-pose['pitch_rad'])) > 1e-7:
                continue
            try:
                tf = buffer.lookup_transform('base_link', 'pitch_link', Time.from_msg(msg.header.stamp))
            except TransformException:
                return
            q = tf.transform.rotation
            expected = (-math.sin(pose['yaw_rad']/2)*math.sin(pose['pitch_rad']/2),
                        math.cos(pose['yaw_rad']/2)*math.sin(pose['pitch_rad']/2),
                        math.sin(pose['yaw_rad']/2)*math.cos(pose['pitch_rad']/2),
                        math.cos(pose['yaw_rad']/2)*math.cos(pose['pitch_rad']/2))
            actual = (q.x, q.y, q.z, q.w)
            if min(max(abs(a-b) for a,b in zip(actual, expected)),
                   max(abs(a+b) for a,b in zip(actual, expected))) > 1e-6:
                continue
            proof['loop_closure'] = dict(planner_receipt=data, applied=pose,
                                         joint_stamp_ns=Time.from_msg(msg.header.stamp).nanoseconds,
                                         joints=joints, tf_quaternion=actual)
            return


subscriptions = [node.create_subscription(typ, topic, callback, qos) for typ, topic, callback, qos in [
    (Armors, '/detector/armors', armors, sensor_qos),
    (Target, args.tracker_topic, target, sensor_qos),
    (GimbalCmd, args.command_topic, command, sensor_qos),
    (String, '/p1sim/command_received', receipt, 10),
    (String, '/p1sim/gimbal_applied', applied_pose, 10),
    (JointState, '/joint_states', joint, 10),
    (Image, '/image_raw', image, sensor_qos),
    (CameraInfo, '/camera_info', info, sensor_qos),
]]
required = {'nonempty_armors', 'tracking', 'control', 'loop_closure', 'source_time_tf'}
deadline = time.monotonic() + args.timeout
try:
    while time.monotonic() < deadline and not required <= proof.keys():
        rclpy.spin_once(node, timeout_sec=0.05)
    nodes = sorted(node.get_node_names())
    proof['nodes'] = nodes
    proof['required_nodes_alive'] = all(n in nodes for n in ('detector', 'rm_tracker', 'rm_planner', 'virtual_serial'))
    proof['hardware_nodes_absent'] = not any(any(x in n for x in (
        'hik_camera', 'mindvision', 'camera_node', 'serial_driver', 'dm_imu', 'fire_node')) for n in nodes)
    proof['pass'] = required <= proof.keys() and proof['required_nodes_alive'] and proof['hardware_nodes_absent']
    Path(args.output).write_text(json.dumps(proof, indent=2)+'\n')
    print(json.dumps(proof, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if proof['pass'] else 1)
