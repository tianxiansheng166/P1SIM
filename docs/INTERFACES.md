# Developer interfaces v1

ROS 2 Humble; DDS domain 88 by default. All sensors are stamped with Gazebo
simulation time. Algorithms must enable `use_sim_time`. Clock restarts on reset.

| Topic | Type | Contract |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo clock, one bridge publisher |
| `/image_raw` | `sensor_msgs/msg/Image` | 640x480 RGB, 15 Hz requested, optical frame, best effort |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Same image stamp, ideal pinhole, 60 degree generic / 90 degree AutoAim horizontal FOV, zero distortion, best effort |
| `/tf` | `tf2_msgs/msg/TFMessage` | Simulation clock stamps, reliable |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Camera optical convention, transient local |
| `/joint_states` | `sensor_msgs/msg/JointState` | yaw/pitch in rad, reliable; kinematic applied command state |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation only, pitch_link, best effort; velocity/acceleration covariance -1 |
| `/p1sim/gimbal_command` | `geometry_msgs/msg/Vector3` | x=yaw, y=pitch in rad; z reserved; absolute angles, reliable |
| `/p1sim/visibility` | `std_msgs/msg/String` | Read-only developer projection from scene geometry, image stamp and confirmed camera angles; never an algorithm input |
| `/p1sim/gimbal_applied` | `std_msgs/msg/String` | JSON after Gazebo pose service succeeds; yaw/pitch rad and simulation time |
| `/p1sim/command_received` | `std_msgs/msg/String` | JSON receipt with monotonic consumed-command sequence and consumption simulation time; receipt does not prove physics application |
| `/armor_planner/cmd_gimbal` | user `rm_interfaces/msg/GimbalCmd` | Optional ABI adapter: configurable topic; best effort, control gate, yaw/pitch rad; fire ignored |

Frames: `odom -> base_link -> yaw_link -> pitch_link -> camera_link ->
camera_optical_frame`. Base is fixed; yaw axis +Z, pitch axis +Y, camera forward
+X. Optical axes follow ROS (+Z forward, +X right, +Y down). Camera origin is
(0,0,1) m relative to base. `odom` is at the initial gimbal pivot: the fixed
`odom -> base_link` translation is (0,0,-1) m. This matches AutoAim's origin
contract; the camera's Gazebo world elevation remains 1 m. Positive pitch points camera down. If the user's
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

The optional AutoAim status includes discovered node names, received armors,
nonempty armors, target/tracking counts, Planner/control counts, latest control
and fire request, command receipts and applied-pose counts. Counters are cumulative;
use callback ages and latest flags to distinguish past success from current tracking.
The base `/health` readiness does not depend on the user algorithm. `fire` is
only observed and recorded; there is no projectile or firing process.

`dev record` also records `/detector/armors`, the configured tracker/Planner
topics, and `/p1sim/gimbal_applied`. Unknown optional topics do not prevent
recording the ordinary simulation topics. See [AutoAim integration](AUTOAIM.md).

The sustained AutoAim profile adds current freshness flags and
`autoaim.gap_age_sim_seconds` (detection, tracking, control; null before first
positive), armor/target IDs and `messages.fire_ignored`. Wall-clock callback
ages remain separate. `dev record` also captures `/p1sim/visibility`. See
[SUSTAINED_AUTOAIM.md](SUSTAINED_AUTOAIM.md) for timing and projection limits.
