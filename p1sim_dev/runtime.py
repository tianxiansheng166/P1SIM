"""Supervise Gazebo, its standard ROS bridge and the developer sensor adapter."""
import functools
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from p1sim_dev.scene import Scene


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class Environment(Node):
    def __init__(self, children, scene):
        super().__init__('p1sim_environment', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.children = children
        self.scene = scene
        self.sim_ns = 0
        self.positive_stamps = {}
        self.visibility = {}
        self.recorder = None
        self.record_log = None
        self.counts = dict(clock=0, image=0, camera_info=0, commands=0, applied=0,
                           armors=0, nonempty_armors=0, targets=0, tracking=0, planner=0, control=0, fire_ignored=0)
        self.autoaim_last = {}
        self.gimbal_topic = os.environ.get("P1SIM_GIMBAL_TOPIC", "/armor_planner/cmd_gimbal")
        self.tracker_topic = os.environ.get("P1SIM_TRACKER_TOPIC", "/tracker/target")
        self.last = {}
        self.yaw = self.pitch = 0.0
        self.command = None
        self.pose_pending = None
        self.pose_applied = None
        self.lock = threading.RLock()
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        optical = self.transform('camera_link', 'camera_optical_frame', 0, (0, 0, 0), quaternion(-math.pi/2, 0, -math.pi/2))
        self.static_tf.sendTransform(optical)
        self.image_pub = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.ack_pub = self.create_publisher(String, '/p1sim/command_received', 10)
        self.visibility_pub = self.create_publisher(String, '/p1sim/visibility', 10)
        self.applied_pub = self.create_publisher(String, '/p1sim/gimbal_applied', 10)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Image, '/p1sim/raw_image', self.image, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/p1sim/gimbal_command', self.control, 10)
        # Optional ABI adapter uses the user's own mounted interface package.
        try:
            from rm_interfaces.msg import GimbalCmd, Armors, Target
        except ImportError:
            self.autoaim_adapter = False
        else:
            self.create_subscription(GimbalCmd, self.gimbal_topic, self.autoaim_control, qos_profile_sensor_data)
            self.create_subscription(Armors, '/detector/armors', self.armors, qos_profile_sensor_data)
            self.create_subscription(Target, self.tracker_topic, self.target, qos_profile_sensor_data)
            self.autoaim_adapter = True
        self.create_timer(0.02, self.apply_command, clock=rclpy.clock.Clock())

    def transform(self, parent, child, stamp, xyz, q):
        msg = TransformStamped()
        if stamp != 0:
            msg.header.stamp = stamp
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        return msg

    def witness(self, name):
        with self.lock:
            self.counts[name] += 1
            self.last[name] = time.monotonic()

    def clock(self, msg):
        self.witness('clock')
        stamp = msg.clock
        self.sim_ns = stamp.sec*10**9 + stamp.nanosec
        self.tf.sendTransform([
            self.transform('odom', 'base_link', stamp, (0, 0, -1), (0., 0., 0., 1.)),
            self.transform('base_link', 'yaw_link', stamp, (0, 0, 1), quaternion(0, 0, self.yaw)),
            self.transform('yaw_link', 'pitch_link', stamp, (0, 0, 0), quaternion(0, self.pitch, 0)),
            self.transform('pitch_link', 'camera_link', stamp, (0, 0, 0), (0., 0., 0., 1.))])
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        joint.position = [self.yaw, self.pitch]
        self.joint_pub.publish(joint)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'pitch_link'
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = quaternion(0, self.pitch, self.yaw)
        # Kinematic orientation only; acceleration and angular rate are unavailable.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

    def image(self, msg):
        msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = msg.width, msg.height
        focal = msg.width / (2 * math.tan(self.scene.fov / 2))
        info.distortion_model = 'plumb_bob'
        info.d = [0.] * 5
        info.k = [focal, 0., msg.width/2, 0., focal, msg.height/2, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [focal, 0., msg.width/2, 0., 0., focal, msg.height/2, 0., 0., 0., 1., 0.]
        self.info_pub.publish(info)
        self.witness('image')
        self.witness('camera_info')
        self.visibility = dict(simulation_time_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            yaw_rad=self.yaw, pitch_rad=self.pitch, diagnostic_only=True,
            targets=self.scene.project(msg.width, msg.height, self.yaw, self.pitch))
        self.visibility_pub.publish(String(data=json.dumps(self.visibility)))

    def armors(self, msg):
        self.witness('armors')
        self.autoaim_last['armor_count'] = len(msg.armors)
        if msg.armors:
            self.witness('nonempty_armors')
            self.positive_stamps['detection'] = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.autoaim_last['armor_ids'] = [a.number for a in msg.armors]

    def target(self, msg):
        self.witness('targets')
        self.autoaim_last['tracking'] = bool(msg.tracking)
        if msg.tracking:
            self.witness('tracking')
            self.positive_stamps['tracking'] = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.autoaim_last['target_id'] = msg.id

    def autoaim_control(self, msg):
        self.witness('planner')
        receipt = dict(topic=self.gimbal_topic, stamp_sec=msg.header.stamp.sec,
                       stamp_nanosec=msg.header.stamp.nanosec, frame_id=msg.header.frame_id,
                       control=bool(msg.control), fire_requested=bool(msg.fire),
                       yaw_rad=msg.yaw, pitch_rad=msg.pitch, fire_supported=False)
        self.autoaim_last['planner'] = receipt
        if msg.fire:
            self.witness('fire_ignored')
        if msg.control:
            self.witness('control')
            self.positive_stamps['control'] = self.sim_ns
            command = Vector3(x=float(msg.yaw), y=float(msg.pitch), z=0.)
            self.control(command, receipt)

    def control(self, msg, receipt=None):
        if not all(math.isfinite(v) for v in (msg.x, msg.y)) or abs(msg.y) > math.pi / 2:
            self.get_logger().warning('Rejected non-finite or out-of-range gimbal command')
            return
        self.command = (msg.x, msg.y)
        self.witness('commands')
        receipt = dict(receipt or {'yaw_rad': msg.x, 'pitch_rad': msg.y, 'fire_supported': False})
        receipt.update(sequence=self.counts['commands'], simulation_time_ns=self.sim_ns)
        self.ack_pub.publish(String(data=json.dumps(receipt)))

    def apply_command(self):
        if self.pose_pending is not None:
            if self.pose_pending.poll() is None:
                return
            output, _ = self.pose_pending.communicate()
            if self.pose_pending.returncode == 0 and 'data: true' in output:
                self.yaw, self.pitch = self.pose_applied
                self.witness("applied")
                self.applied_pub.publish(String(data=json.dumps(dict(yaw_rad=self.yaw, pitch_rad=self.pitch,
                    simulation_time_ns=self.get_clock().now().nanoseconds, fire_supported=False))))
            self.pose_pending = None
        if self.command is None or self.command == (self.yaw, self.pitch):
            return
        yaw, pitch = self.command
        x, y, z, w = quaternion(0, pitch, yaw)
        request = f'name: "camera", position: {{x: 0, y: 0, z: 1}}, orientation: {{x:{x}, y:{y}, z:{z}, w:{w}}}'
        self.pose_applied = self.command
        self.pose_pending = subprocess.Popen(['ign', 'service', '-s', '/world/developer/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000',
            '--req', request], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def status(self):
        with self.lock:
            age = {key: round(time.monotonic()-value, 3) for key, value in self.last.items()}
            processes = {name: proc.poll() is None for name, proc in self.children.items()}
            ready = all(processes.values()) and all(age.get(key, 99) < 5 for key in ('clock', 'image', 'camera_info'))
            gaps = {k: None if k not in self.positive_stamps else max(0, (self.sim_ns-self.positive_stamps[k])/1e9)
                    for k in ('detection', 'tracking', 'control')}
            fresh = {k: age.get(source, 99) < 2 and gaps[k] is not None and gaps[k] <= .5
                     for k, source in (('detection', 'nonempty_armors'), ('tracking', 'tracking'), ('control', 'control'))}
            fresh['detection'] &= self.autoaim_last.get('armor_count', 0) > 0
            fresh['tracking'] &= self.autoaim_last.get('tracking', False)
            fresh['control'] &= self.autoaim_last.get('planner', {}).get('control', False)
            return dict(ready=ready, scene_profile=os.environ.get('P1SIM_SCENE', 'developer'), simulation_time_ns=self.sim_ns, processes=processes, messages=dict(self.counts), age_seconds=age,
                        gimbal_rad=dict(yaw=self.yaw, pitch=self.pitch), autoaim_adapter=self.autoaim_adapter,
                        autoaim=dict(command_topic=self.gimbal_topic, tracker_topic=self.tracker_topic,
                                     nodes=sorted(self.get_node_names()), latest=dict(self.autoaim_last),
                                     gap_age_sim_seconds=gaps, fresh=fresh,
                                     visibility=self.visibility),
                        recording_directory='/data/bags', recording=self.recorder is not None and self.recorder.poll() is None, mode='developer')


    def start_record(self, name):
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', name):
            raise ValueError('Invalid recording name')
        with self.lock:
            if self.recorder is not None:
                raise ValueError('Stop the current recorder first')
            output = Path('/data/bags') / name
            if output.exists():
                raise ValueError('Recording already exists; choose a new name')
            self.record_log = open('/data/recording.log', 'a')
            self.recorder = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', str(output),
                '/clock', '/image_raw', '/camera_info', '/tf', '/tf_static', '/joint_states',
                '/imu/data', '/p1sim/visibility', '/p1sim/gimbal_command', '/p1sim/command_received', '/p1sim/gimbal_applied',
                '/detector/armors', self.tracker_topic, self.gimbal_topic],
                stdout=self.record_log, stderr=subprocess.STDOUT, start_new_session=True)
            return {'recording': name}

    def stop_record(self):
        with self.lock:
            if self.recorder is None:
                raise ValueError('No active recorder')
            if self.recorder.poll() is None:
                os.killpg(self.recorder.pid, signal.SIGINT)
            try:
                code = self.recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.recorder.pid, signal.SIGTERM)
                code = self.recorder.wait(timeout=2)
            self.recorder = None
            self.record_log.close()
            return {'recording_exit': code}


class View(BaseHTTPRequestHandler):
    environment = None

    def do_GET(self):
        status = self.environment.status()
        if self.path in ('/status', '/health'):
            payload = json.dumps(status).encode()
            self.send_response(503 if self.path == '/health' and not status['ready'] else 200)
            self.send_header('Content-Type', 'application/json')
        else:
            payload = b'''<!doctype html><meta charset="utf-8"><title>P1SIM Live</title>
<style>body{font:18px monospace;background:#15202b;color:#d8edff;margin:3em}pre{line-height:1.6}</style>
<h1>P1SIM Developer Simulation</h1><p id="loop">Waiting for live state</p><pre id="state"></pre>
<script>async function update(){try{const s=await(await fetch('/status')).json();const a=s.autoaim;loop.textContent=['detection','tracking','control'].map(k=>k+': '+(a.fresh[k]?'ACTIVE':'WAIT / LOST')+' (gap '+(a.gap_age_sim_seconds[k]===null?'never':a.gap_age_sim_seconds[k].toFixed(2)+' sim s')+')').join(' | ');state.textContent=JSON.stringify(s,null,2)}catch(e){state.textContent='Disconnected'}}update();setInterval(update,1000)</script>'''
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size < 1024:
                raise ValueError('Invalid request size')
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError('Expected JSON object')
            if self.path == '/record/start':
                result = self.environment.start_record(payload.get('name'))
            elif self.path == '/record/stop':
                result = self.environment.stop_record()
            else:
                raise ValueError('Unknown operation')
            self.send_response(200)
        except (ValueError, OSError) as error:
            result = {'error': str(error)}
            self.send_response(400)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode())

    def log_message(self, *args):
        pass


def main():
    Path(os.environ.get('HOME', '/tmp/p1sim-home')).mkdir(parents=True, exist_ok=True)
    Path('/data/bags').mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('IGN_PARTITION', 'p1sim-developer')
    children = {}
    logs = []
    rclpy.init()
    node = None
    try:
        profile = os.environ.get('P1SIM_SCENE', 'developer')
        if profile not in ('developer', 'autoaim'):
            raise ValueError('P1SIM_SCENE must be developer or autoaim')
        world_path = '/opt/p1sim/worlds/' + profile + '.sdf'
        scene = Scene(world_path)
        commands = {
            'gazebo': ['ign', 'gazebo', '--headless-rendering', '-r', '-s', world_path],
            'ros_bridge': ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '--ros-args', '-r', '/camera/image:=/p1sim/raw_image']}
        for name, command in commands.items():
            log = open('/data/' + name + '.log', 'a', buffering=1)
            logs.append(log)
            children[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = Environment(children, scene)
        View.environment = node
        server = ThreadingHTTPServer(('0.0.0.0', 8765), View)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node and node.recorder is not None:
            node.stop_record()
        if node and node.pose_pending and node.pose_pending.poll() is None:
            node.pose_pending.terminate()
            node.pose_pending.wait(timeout=3)
        for proc in children.values():
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in children.values():
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for log in logs:
            log.close()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
