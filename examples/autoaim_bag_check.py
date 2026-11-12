#!/usr/bin/env python3
"""Audit sustained developer use from a real bag, with no runtime publications."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from p1sim_dev.runtime import quaternion


def stamp(msg):
    return msg.sec + msg.nanosec / 1e9


def command_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
            bool(msg.control), msg.yaw, msg.pitch, bool(msg.fire))


def receipt_key(data):
    return (data['stamp_sec'], data['stamp_nanosec'], data['frame_id'],
            data['control'], data['yaw_rad'], data['pitch_rad'], data['fire_requested'])


def max_gap(times, start, end):
    # Include both window edges, including a possibly empty trailing outage.
    selected = sorted(set([start, end] + [t for t in times if start <= t <= end]))
    return max(b-a for a, b in zip(selected, selected[1:]))


def audit(uri, window, max_outage, output):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts = Counter()
    positive = {k: [] for k in ('detection', 'tracking', 'control')}
    timeline = []
    commands = Counter()
    receipts, applied, joints, transforms, images, visibility = [], [], [], [], [], []
    sim_time = 0.
    fire = 0
    first, previous = None, None
    image_changes = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        counts[topic] += 1
        msg = deserialize_message(raw, types[topic])
        if topic == '/clock':
            sim_time = stamp(msg.clock)
        elif topic == '/image_raw':
            t = stamp(msg.header.stamp)
            im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                im = im[:, :, ::-1]
            digest = hashlib.sha256(im.tobytes()).hexdigest()
            # Independent visual witness: red target pixels must remain away
            # from all frame edges. This mask is never an algorithm input.
            red = (im[:, :, 0].astype(int) > im[:, :, 1].astype(int)+40) & (im[:, :, 0].astype(int) > im[:, :, 2].astype(int)+40)
            ys, xs = np.where(red)
            bounds = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())] if len(xs) else None
            images.append((t, digest, bounds, msg.width, msg.height))
            if previous is not None and previous[1] != digest:
                image_changes.append((previous[0], t))
            if first is None:
                first = im.copy()
            last = im.copy()
            previous = (t, digest)
        elif topic == '/detector/armors':
            t = stamp(msg.header.stamp)
            timeline.append(dict(time=t, armors=[a.number for a in msg.armors]))
            if msg.armors:
                positive['detection'].append(t)
        elif topic.endswith('/target'):
            t = stamp(msg.header.stamp)
            timeline.append(dict(time=t, tracking=bool(msg.tracking), id=msg.id))
            if msg.tracking:
                positive['tracking'].append(t)
        elif topic.endswith('/cmd_gimbal'):
            commands[command_key(msg)] += 1
            if msg.control:
                positive['control'].append(sim_time)
            fire += bool(msg.fire)
        elif topic == '/p1sim/command_received':
            receipts.append((sim_time, json.loads(msg.data)))
        elif topic == '/p1sim/gimbal_applied':
            applied.append(json.loads(msg.data))
        elif topic == '/p1sim/visibility':
            visibility.append(json.loads(msg.data))
        elif topic == '/joint_states':
            joints.append((stamp(msg.header.stamp), dict(zip(msg.name, msg.position))))
        elif topic == '/tf':
            transforms.extend(msg.transforms)
    start = min(positive['tracking']) if positive['tracking'] else None
    end = start + window if start is not None else None
    result = dict(bag=uri, first_tracking_time=start, required_window_seconds=window,
                  final_sim_time=sim_time, topic_counts=dict(counts), fire_requests_ignored=fire,
                  positive_counts={k: len(v) for k, v in positive.items()}, timeline=timeline)
    exact = Counter(receipt_key(d) for _, d in receipts)
    unmatched = sum((exact-commands).values())
    sequences = [d.get('sequence') for _, d in receipts]
    sequence_complete = sequences == list(range(1, len(receipts)+1))
    result.update(receipt_sequence_complete=sequence_complete, receipts=len(receipts), exact_receipts=len(receipts)-unmatched,
                  receipt_fire_unsupported=all(d['fire_supported'] is False for _, d in receipts), applications=applied)
    feedback = []
    for a in applied:
        t = a['simulation_time_ns']/1e9
        matching_joint = any(s >= t and abs(j.get('gimbal_yaw_joint', math.inf)-a['yaw_rad']) < 1e-7 and
                             abs(j.get('gimbal_pitch_joint', math.inf)-a['pitch_rad']) < 1e-7 for s, j in joints)
        # Runtime publishes yaw and pitch as two links; verify each component.
        matching_tf = all(any(stamp(tf.header.stamp) >= t and tf.child_frame_id == child and
            min(max(abs(v-e) for v, e in zip((tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w), q)),
                max(abs(v+e) for v, e in zip((tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w), q))) < 1e-7
            for tf in transforms) for child, q in (('yaw_link', quaternion(0, 0, a['yaw_rad'])), ('pitch_link', quaternion(0, a['pitch_rad'], 0))))
        feedback.append(matching_joint and matching_tf)
    result['every_application_joint_tf'] = bool(feedback) and all(feedback)
    if start is not None:
        gaps = {k: max_gap(v, start, end) for k, v in positive.items()}
        vis = [v for v in visibility if start <= v['simulation_time_ns']/1e9 <= end]
        frames = [im for im in images if start <= im[0] <= end]
        visible = bool(vis) and all(v['targets'] and all(t['fully_in_fov'] for t in v['targets']) for v in vis)
        rendered_visible = bool(frames) and all(b is not None and b[0] > 0 and b[1] > 0 and b[2] < width-1 and b[3] < height-1 for _, _, b, width, height in frames)
        changes = [a for a in applied if start <= a['simulation_time_ns']/1e9 <= end and
            any(a['simulation_time_ns']/1e9-.3 <= before <= after <= a['simulation_time_ns']/1e9+.3 for before, after in image_changes)]
        result.update(sustained_window=[start, end], max_gap_sim_seconds=gaps, visibility_samples=len(vis),
                      fully_in_fov=visible, rendered_target_interior=rendered_visible,
                      command_induced_image_changes=len(changes), window_image_samples=len(frames),
                      frame_red_bounds=[dict(time=t, bounds=b) for t, _, b, _, _ in frames])
    required = ('/clock', '/image_raw', '/camera_info', '/detector/armors', '/tracker/target',
                '/armor_planner/cmd_gimbal', '/p1sim/command_received', '/p1sim/gimbal_applied', '/p1sim/visibility', '/joint_states', '/tf', '/tf_static')
    result['pass'] = (start is not None and sim_time >= end and all(counts[k] > 0 for k in required) and
        all(g <= max_outage for g in gaps.values()) and visible and rendered_visible and bool(changes) and
        bool(receipts) and sequence_complete and unmatched == 0 and receipts[-1][1].get('simulation_time_ns', 0)/1e9 >= end and result['receipt_fire_unsupported'] and result['every_application_joint_tf'])
    dest = Path(output)
    if first is not None:
        cv2.imwrite(str(dest.with_suffix('.before.png')), first[:, :, ::-1])
        cv2.imwrite(str(dest.with_suffix('.after.png')), last[:, :, ::-1])
    dest.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('timeline', 'frame_red_bounds', 'applications')}, indent=2))
    return result['pass']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--window', type=float, default=5.)
    parser.add_argument('--max-outage', type=float, default=.5)
    parser.add_argument('--output', default='/data/sustained.json')
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.bag, args.window, args.max_outage, args.output) else 1)
