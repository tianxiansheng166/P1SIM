# P1SIM Foundation 1.0 — Developer Edition

A local ROS 2 / Gazebo environment for developing algorithms. Start the world,
launch your own ROS software, observe sensors, record a bag, and stop.
No task registration or research qualification is needed for normal developer use.

**中文快速入门：** [从零安装到运行自己的算法](docs/QUICKSTART.zh-CN.md) ·
[接入自己的 AutoAim 工作区](docs/USER_AUTOAIM.md) ·
[Foundation 接口与扩展](docs/FOUNDATION.md)

This public distribution is a self-build **CPU-headless developer simulation**.
It does not download a private container image, model, ROS workspace, or
experimental ResultPack. It is not a competition-physics/scorer product.
The first public Foundation package is a preview of the independently
versioned source distribution; a second-host release acceptance is still open.

## Quickstart

Prerequisites: Linux x86_64, Docker Engine with Compose v2 and permission to run
Docker, Python 3.10+, Internet for the first build. Allow 6 GB free disk and
4 GB RAM. The default uses Mesa software rendering; no GPU is required.
ROS 2 Humble and Gazebo Fortress run inside the container; host ROS is optional.
If upstream apt downloads are slow in your region, before bootstrap set
`P1SIM_UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu` and
`P1SIM_ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`.
Package signatures remain verified with the upstream keys.
Other host architectures and Docker Desktop are not validated.

```bash
git clone https://github.com/tianxiansheng166/P1SIM.git
cd P1SIM
./p1sim dev bootstrap
./p1sim dev doctor
./p1sim dev up
./p1sim dev run -- python3 /opt/p1sim/examples/smoke.py
./p1sim dev run -- ros2 launch /opt/p1sim/examples/consumer.launch.py
```

The last command runs the standard `robot_state_publisher` on the simulation's
joint state. Ctrl-C stops that consumer. View live health and message counters at
<http://localhost:8765>, or use `./p1sim dev status` and ordinary ROS tools:

```bash
./p1sim dev run -- ros2 topic list
./p1sim dev run -- ros2 topic hz /image_raw
./p1sim dev record experiment-1  # Ctrl-C finalizes the ROS bag
./p1sim dev reset                # restart world, clock and adapters together
./p1sim dev down
```

Bags and Gazebo/bridge logs stay in `~/.local/state/p1sim-developer`.
Copy that directory to export a run. Logs are not deleted on stop.
Reset restarts the clock at zero: stop and restart your algorithm and recorder
around reset if they do not handle clock jumps.

## Your normal AutoAim workspace

Mount your own built Humble workspace; binaries and weights stay on your machine.
For a workspace built against the same Ubuntu 22.04 / Humble ABI:

```bash
export P1SIM_USER_WORKSPACE=/absolute/path/to/your/workspace
export P1SIM_WORKSPACE_SETUP=/user_ws/install/setup.bash
./p1sim dev up
./p1sim dev shell
# in the shell: run your ordinary ros2 launch with use_sim_time:=true
```

Set these variables before the first `up`; run `down` then `up` after changing them.
The mount is read-only. Write algorithm output to `/data`, not `/user_ws`.
You can alternatively attach a host Humble workspace using `ROS_DOMAIN_ID=88`
and standard DDS networking. Container-shell use is the supported simple path;
host DDS discovery depends on your Docker network and firewall configuration.

Use the algorithm's existing camera-free launch or override at the sensor
boundary. Disable hardware camera/serial/IMU nodes; enable `use_sim_time` for all
algorithm nodes and remap their Image, CameraInfo and command topics as needed.
Do not run a second TF authority for the same simulation frames.

If the mounted workspace provides `rm_interfaces/msg/GimbalCmd`, the optional
adapter subscribes to `/armor_planner/cmd_gimbal` by default (override using
`P1SIM_GIMBAL_TOPIC`; radian yaw/pitch commands enabled by
`control`). Set `P1SIM_WORKSPACE_SETUP` before `up` to load that ABI. `fire` is
ignored: this development world has no weapon or projectile path. See
[interfaces](docs/INTERFACES.md) before connecting a different stack version.
The full private detector/tracker stack and proprietary weights are not bundled.

## Foundation V1: scenarios, batch and extensions

The original `dev bootstrap/doctor/up/run/record/reset/down` flow is unchanged.
The Foundation adds versioned config/seed/epoch, standard platform observables
and an extension API. Try the built-in scenes and bounded headless batch:

```bash
P1SIM_SCENE=autoaim ./p1sim dev up
./p1sim dev status
./p1sim dev down
./p1sim dev batch --config configs/extensions.json --seed 42 --output ./results/example-42
```

Batch output is a **platform execution receipt**, not a scientific evaluation
ResultPack. The `autoaim` scene is a developer target presentation and does not
include detector binaries or private weights. See
[Foundation configuration and plugin contract](docs/FOUNDATION.md) and the
[sample external caller](examples/batch_caller.py).

## Commands and configuration

`./p1sim dev run -- <command>` executes ordinary commands inside the ROS environment.
`./p1sim dev shell` opens that environment interactively. `logs` shows recent
container output; detailed server and bridge logs live in the state directory.

Optional environment variables:

| Variable | Default | Purpose |
|---|---|---|
| `P1SIM_STATE_DIR` | `~/.local/state/p1sim-developer` | Logs and bags |
| `P1SIM_STATUS_PORT` | `8765` | Local live view |
| `P1SIM_ROS_DOMAIN_ID` | `88` | ROS graph isolation |
| `P1SIM_PROJECT` | `p1sim-dev` | Compose project name |
| `P1SIM_USER_WORKSPACE` | bundled examples | User workspace mount |
| `P1SIM_WORKSPACE_SETUP` | unset | Container path to user ROS setup |
| `P1SIM_IMAGE` | `p1sim-foundation:1.0.0` | Optional compatible image you build locally |
| `P1SIM_SCENE` | `developer` | `developer` or `autoaim` |
| `P1SIM_GIMBAL_TOPIC` | `/armor_planner/cmd_gimbal` | Optional gimbal command input |

Use a unique project, port, domain and state directory for each simultaneous
instance. For repeatable comparisons, reset between runs and record your source
revision, parameters and bags. V1 ships a generic developer scene and one
single-target AutoAim presentation. It does not promise
bitwise deterministic sensor scheduling or a scoring framework.

## Troubleshooting

`doctor` validates Docker and installed ROS/Gazebo dependencies. If the runtime
image is missing, run `bootstrap`. `up` waits for live clock and camera data;
a failed start is a product error and returns nonzero. Inspect `logs` and the
state directory; software-rendered camera initialization can take longer on
small CPUs. The live view is loopback-only on the host.

The base image and apt dependencies are open source. First build downloads
Ubuntu/ROS packages. Rebuilding later can resolve newer apt package revisions;
retain your image digest for exact binary reproduction. See `release.json`,
`SHA256SUMS` and `THIRD_PARTY_NOTICES.md` for source and dependency boundaries.

See [release scope and limits](docs/RELEASE_NOTES.md) for public preview boundaries.

## Scope

The default is **Developer Simulation**. Lightweight repeatable tests can use
ordinary launches and ROS bags without additional gates. Scientific physics,
provenance and scorer qualification are separate opt-in workflows; no such
qualification is claimed for this basic world. The gimbal adapter is kinematic,
not a calibrated actuator model. The scene contains project-authored geometric
objects, not competition meshes or proprietary detector assets.
