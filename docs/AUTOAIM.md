# Real AutoAim integration

This route uses your own PinyVision workspace and models. P1SIM supplies the
camera, calibration, clock, TF, kinematic gimbal and message receipts. Detector,
Tracker and Planner remain your normal software, with their production parameters.

## Prepare the consumer

The observed source baseline is PinyVision main
`c37eca1057d6a78945ec3ee8c9840db837411340`. The bringup-only integration is
[commit f380dde](https://github.com/tianxiansheng166/PinyVision/commit/f380dde)
on `integration/t229-external-camera-v1`. Use that revision, or carry the same
external-camera launch boundary into your workspace. Algorithm source is
unchanged from the baseline. The existing hardware launch remains the default.

Build a Linux x86_64 / Ubuntu 22.04 / ROS Humble workspace with current
`rm_interfaces`, `rm_tools`, `rm_auto_aim_common`, `detector`, `rm_tracker`,
`rm_planner`, `rm_serial_driver` and `rm_vision_bringup`. Install all their
package.xml dependencies; the Planner requires `ros-humble-angles`. Build the
external bringup after the binaries. Do not mix an older `armor_detector_node`
with this current `detector_node` composition.

The current Detector looks up the `pinyvision_models` package and reads
`models/armor/0526.xml` plus its BIN. Supply your authorized private model
package. If model repository access is unavailable, an owner-authorized local
installation can provide the same package layout. Local validation used the
owner's already installed 0526 assets; the private submodule revision could not
be fetched, so identity with that submodule's weights is not claimed.
No source, models or training data from your workspace enter the P1SIM image.

## Use a compatible runtime image

The default P1SIM image supplies ROS/Gazebo, not every private algorithm's
libraries. Start with your own Humble dependency image containing OpenVINO,
OpenCV, Ceres, fmt and your ROS dependencies, then add the simulator:

```bash
docker build -f Dockerfile.consumer \
  --build-arg AUTOAIM_IMAGE=my-autoaim-dependencies:humble \
  -t p1sim-autoaim:local .
```

`UBUNTU_MIRROR` and `ROS_MIRROR` build arguments are supported, as in the base
Dockerfile. The observed runtime used CPU OpenVINO 2023.3. The original model
runs on CPU; no GPU, camera device or serial device is needed. Your installed
binaries must match the OpenVINO and Ubuntu ABI in your image. This optional
image extends your dependencies; it does not distribute PinyVision or weights.
Keep a dependency-only image if you intend to share it publicly.

If OpenVINO requires a setup script, create a wrapper in your own workspace:

```bash
# /absolute/path/to/my-workspace/simulation_setup.bash
source /opt/intel/openvino/setupvars.sh  # use your image's actual location
source /user_ws/install/setup.bash
```

Otherwise point directly to `/user_ws/install/setup.bash`.

## Launch and observe

```bash
export P1SIM_IMAGE=p1sim-autoaim:local
export P1SIM_USER_WORKSPACE=/absolute/path/to/my-workspace
export P1SIM_WORKSPACE_SETUP=/user_ws/simulation_setup.bash
export P1SIM_GIMBAL_TOPIC=/armor_planner/cmd_gimbal
./p1sim dev up
./p1sim dev shell
# Inside the shell:
ros2 launch rm_vision_bringup vision_bringup.launch.py camera:=external inference_device:=CPU
```

The workspace is mounted read-only, including its weights. Write output to
`/data`. Configure environment variables before `up`; restart after changing
them. Build your optional image explicitly before setting `P1SIM_IMAGE`;
`dev bootstrap` builds the default Dockerfile and should not overwrite it.

The external camera mode enables simulation time and uses the original
Detector, Tracker, Planner and virtual serial binaries and YAML settings.
Only the inference execution device changes. Hardware camera/serial/IMU nodes,
buff nodes and automatic recording are omitted. P1SIM owns `/tf` and
`/tf_static`; virtual serial's synthetic TF goes to `/virtual_serial/tf`.
Its existing warning about an absent `buff_detector` is expected.

From another terminal with the same environment:

```bash
./p1sim dev record autoaim-run
# Ctrl-C finalizes the bag, including real detector/tracker/Planner topics.
./p1sim dev run -- python3 /opt/p1sim/examples/autoaim_check.py
./p1sim dev status
```

Start the observer and recorder before launching AutoAim to capture startup
acquisition. The observer only subscribes: it never injects armors, targets or
commands. It checks nonempty armors, tracking, enabled Planner output, an exact
matching command receipt, Gazebo application, subsequent matching joints and
TF, and source-image-time TF. The JSON result is `/data/autoaim-check.json`.
It times out if the current graph/scene does not produce the required evidence.

The live page at <http://localhost:8765> shows discovered nodes, message counts,
callback ages, latest tracking/control/fire flags and applied yaw/pitch. Counters
are cumulative; stale positive counts alone do not mean tracking is still active.
`fire` requests are observable, but P1SIM ignores them. There is no firing or
projectile process. Commands are absolute radian angles with sensor-data QoS.

A configured PinyVision namespace also namespaces `tracker/target` and the
Planner command. Set `P1SIM_TRACKER_TOPIC` and `P1SIM_GIMBAL_TOPIC` to their
absolute names and pass matching observer topic options. Detector's armor
and CameraInfo topics remain absolute in the current source.

Ctrl-C ends the launch; `./p1sim dev down` removes the container and its processes.
Logs and bags remain in the state directory. Restart algorithms around `reset`,
because the simulation clock jumps back to zero.

## Observed result and limits

[Compact observed messages and loop evidence](autoaim-observed.json) includes
source and asset hashes, exact command receipt matching and the second start.

Two clean starts produced real nonempty Detector output, tracking=true,
control=true and movement of the simulated gimbal with corresponding joints/TF.
One short bag contained 10 nonempty armor messages, 7 tracking targets, 163
control commands and 163 exactly matched receipts; 84 Planner messages requested
fire, which remained unsupported. Three Gazebo pose applications were observed.
Image content changed after the camera moved. Hardware nodes were absent.
Ctrl-C removed the consumer, and down removed the container/network.

The scene uses newly authored box geometry for red light bars, white cores and
a seven-segment digit. It contains no competition meshes or training images.
Detection was transient: sustained stable tracking after camera motion is not
established. This validates the real software loop for this source/model pair,
not detector robustness, physical/ballistic accuracy, scoring or hardware readiness.
