#!/usr/bin/env python3
"""Read actual ROS platform messages; exercise dynamics without an algorithm."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='/data/foundation-check.json')
args = parser.parse_args()
rclpy.init()
node = Node('foundation_observer')
state, timing, applications, visibility, images, joints, receipts = [], [], [], [], [], [], []
subscriptions = []
for topic, destination in [('/p1sim/state',state),('/p1sim/camera_timing',timing),
    ('/p1sim/gimbal_applied',applications),('/p1sim/visibility',visibility),('/p1sim/command_received',receipts)]:
    subscriptions.append(node.create_subscription(String,topic,lambda msg,d=destination:d.append(json.loads(msg.data)),10))
subscriptions.append(node.create_subscription(Image,'/image_raw',lambda msg:images.append((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,any(msg.data))),qos_profile_sensor_data))
subscriptions.append(node.create_subscription(JointState,'/joint_states',lambda msg:joints.append(dict(q=list(msg.position),v=list(msg.velocity))),10))
pub=node.create_publisher(Vector3,'/p1sim/gimbal_command',10)
result = {}
try:
    end=time.monotonic()+45
    sent = False
    next_publish = 0.
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if not receipts and pub.get_subscription_count()>0 and time.monotonic() >= next_publish:
            pub.publish(Vector3(x=.4,y=-.3,z=0.))
            sent=True
            next_publish=time.monotonic()+.2
        if applications and state and state[-1]['simulation_time_ns'] > applications[0]['application_time_ns']+3000000000:
            break
    assert sent and receipts and applications and state and timing and images and joints, 'Missing command/sensor/application witnesses'
    receipt=receipts[0]
    assert all(a['application_time_ns']-a['receipt_time_ns']>=100000000 for a in applications)
    assert any(any(abs(v)>0 for v in j['v']) for j in joints)
    assert any(j['v']==[0.,0.] and abs(j['q'][0])>.1 for j in joints), 'Stationary JointState velocity did not clear'
    assert all(abs(v)<=.500001 for s in state for v in s['gimbal_model']['v_rad_s'])
    assert all(abs(q)<=.500001 for s in state for q in s['gimbal_model']['q_rad'])
    assert any(t['delivery_time_ns']>t['source_time_ns'] for t in timing)
    black={ns for ns,nonzero in images if not nonzero}
    occluded={v['simulation_time_ns'] for v in visibility if v['occluded']}
    assert black & occluded, 'No raster blackout at an occluded source stamp'
    assert any(v['targets'] for v in visibility)
    result=dict(passed=True,identity=state[-1],receipt=receipt,first_application=applications[0],
        last_application=applications[-1],application_count=len(applications),timing_samples=len(timing),
        blackout_source_stamps=sorted(black & occluded),joint_velocity_observed=True,
        rate_limit_observed=True,delay_observed=True)
except Exception as error:
    result=dict(passed=False,error=str(error),counts=dict(states=len(state),receipts=len(receipts),applications=len(applications),images=len(images)))
finally:
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps(dict(passed=result['passed'],output=args.output)))
raise SystemExit(0 if result['passed'] else 1)
