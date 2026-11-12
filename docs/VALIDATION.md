# Developer Edition V1 validation

Validated on 2026-11-12: Linux x86_64, Docker Engine, Compose v2.40.3,
CPU/Mesa headless rendering, ROS 2 Humble and Gazebo Fortress 6.18.0.
The Dockerfile builds from the public, digest-pinned ROS base image and upstream
apt packages. No private runtime image, algorithm binary or weight is needed.

Observed results:

- Bootstrap, doctor and up: exit 0.
- Live HTTP view: 200; health observes actual recent ROS callbacks.
- Image + CameraInfo + clock + joint state flow: passed.
- TF lookup at image source timestamps: passed.
- Command (yaw=0.1 rad, pitch=-0.05 rad): camera pose and joint state moved.
- Unmodified upstream `robot_state_publisher` through the supplied ordinary
  ROS launch consumed `/joint_states`; its independent TF chain reproduced
  yaw=0.1 and pitch=-0.05 rad. Ctrl-C removed the consumer from the graph.
- Record: exit 0, finalized rosbag2 database, 7.07 s simulator time, 5,872 messages,
  including 103 images, 104 CameraInfo, 3 command receipts and static TF.
- Reset and its smoke: exit 0.
- Down, second clean up, second smoke, final down: exit 0.

Reproduce with `python3 tests/acceptance.py acceptance.json` after bootstrap.
Use a fresh state directory or unique recording names. The test starts/stops
the environment and records one short bag. It leaves the bag in the state
folder; inspect with `./p1sim dev run -- ros2 bag info /data/bags/<name>` while up.

The representative consumer is upstream ROS software, not the private full
AutoAim detector/tracker/planner launch. The optional mounted `GimbalCmd`
adapter's private package ABI was inspected from the current source contract,
but was not exercised in this public clean-install test. No private detector,
weights, competition-scene behavior, actuator dynamics or scientific accuracy
claim is made.

Installation fixes included readable source permissions for a non-root runtime
and upgrading the complete installed rosbag2 package family together, avoiding
CLI/Python/C++ ABI mixing inherited from the pinned base image.
