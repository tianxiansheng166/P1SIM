# Developer interfaces v1

ROS 2 Humble; DDS domain 88 by default. All sensors are stamped with Gazebo
simulation time. Algorithms must enable `use_sim_time`. Clock restarts on reset.

| Topic | Type | Contract |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo clock, one bridge publisher |
| `/image_raw` | `sensor_msgs/msg/Image` | 640x480 RGB, 15 Hz requested, optical frame, best effort |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Same image stamp, ideal pinhole, 60 degree horizontal FOV, zero distortion, best effort |
| `/tf` | `tf2_msgs/msg/TFMessage` | Simulation clock stamps, reliable |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Camera optical convention, transient local |
| `/joint_states` | `sensor_msgs/msg/JointState` | yaw/pitch in rad, reliable; kinematic applied command state |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation only, pitch_link, best effort; velocity/acceleration covariance -1 |
| `/p1sim/gimbal_command` | `geometry_msgs/msg/Vector3` | x=yaw, y=pitch in rad; z reserved; absolute angles, reliable |
| `/p1sim/command_received` | `std_msgs/msg/String` | JSON receipt; receipt does not prove physics application |
| `/cmd_gimbal` | user `rm_interfaces/msg/GimbalCmd` | Optional ABI adapter: control gate, yaw/pitch rad; fire ignored |

Frames: `odom -> base_link -> yaw_link -> pitch_link -> camera_link ->
camera_optical_frame`. Base is fixed; yaw axis +Z, pitch axis +Y, camera forward
+X. Optical axes follow ROS (+Z forward, +X right, +Y down). Camera origin is
(0,0,1) m relative to base. Positive pitch points camera down. If the user's
command convention differs, add an explicit conversion at that boundary;
do not silently invert an algorithm command.

Command pose changes go through Gazebo's `/world/developer/set_pose` service.
V1 teleports the camera model; it has no inertia, rate/torque limit or actuator
latency model. Joint/TF state reflects the service-completed requested pose.
There can be a transition image during command application. This is a local
development interface, not an exact sensor/actuator synchronization authority.

The live page and `/health` endpoint check server/bridge process liveness and
recent clock/image/CameraInfo callbacks (5 second freshness). Smoke additionally
requires TF at actual source image stamps, joint movement and command receipts.
Readiness does not imply a private algorithm is present or correctly configured.

The unchanged representative consumer is ROS `robot_state_publisher` through
`examples/consumer.launch.py`; it subscribes directly to `/joint_states` and
publishes distinct `consumer_*` frames. It does not replace the simulation TF.
