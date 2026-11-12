# P1SIM Foundation V1

Foundation 1.0.0, Developer ROS interface 1, config schema 1 and Python plugin
API 1. Release candidate: `v1.0.0-rc.1`. Developer simulation facts are not
scientific truth qualification, algorithm evaluation or actuator fidelity.

## Run a configuration

The existing `./p1sim dev up` commands and topics continue to work. To select
an explicit configuration, first stop the previous environment, then use:

```bash
./p1sim dev up --config configs/extensions.json
./p1sim dev status
./p1sim dev reset
./p1sim dev down
./p1sim dev batch --config configs/extensions.json --seed 42 --output ./my-run
python3 examples/batch_caller.py --config configs/extensions.json --output ./three-runs
```

`batch` creates a new output directory, unique Compose project and localhost
HTTP port. It runs Gazebo with Mesa CPU headless rendering, waits for READY,
observes a bounded **simulator-time** duration, and removes its container and
network in `finally`, including Ctrl-C/SIGTERM paths. It does not remove the
output directory or the read-only mounted workspace. `--ready-timeout` overrides
the config's wall-time startup timeout. A bounded run also has a wall-time
watchdog of max(120 s, duration × 30). For externally controlled runs, set
`duration_seconds: null` and supply `--stop-file /absolute/path`; the caller
creates that file to finish. The command exits zero only for COMPLETED with
verified teardown. A preexisting output is rejected. Different simultaneous
ROS runs need different `P1SIM_ROS_DOMAIN_ID` values. Port assignment is checked
by Compose at binding; a rare port race fails and tears down safely.

Artifacts are `result.json`, `ready.json`, `final.json`, sampled
`observables.jsonl`, and `runtime/{run.json,manifest.json,lifecycle.jsonl}`.
`result.json` is a platform execution status, not a business ResultPack.
With `recording: true`, bags are in `runtime/bags/batch`. Logs stay under the
run directory. Failed runs preserve their logs and return a nonzero exit.

## Configuration schema 1

JSON objects are canonicalized with sorted keys, explicit defaults and finite
numbers; SHA-256 of the canonical configuration is `config_digest` and
`run_key`. Unknown fields, incompatible versions, invalid seed, non-finite
numbers, malformed topic mappings and unsupported built-in model options are
rejected. No implicit schema migration is performed.

| Field | Default / meaning |
|---|---|
| `config_version`, `interface_version`, `plugin_api_version` | integer 1 |
| `foundation_version` | string `1.0.0` |
| `scene` | `developer`; `autoaim` is the accepted sustained consumer scene |
| `seed` | integer 0 through 2^63−1; default 0 |
| `duration_seconds` | null (external stop) or positive simulation duration |
| `headless` | true; batch requires true; false starts local Gazebo GUI rendering |
| `ready_timeout_seconds` | 120 wall seconds |
| `namespace` | empty; prefix for mapped ordinary sensor/control topics |
| `topics` | absolute overrides: image, camera_info, joint_states, imu, gimbal_command, planner, tracker |
| `recording` | false; automatic bag recording in batch |
| `workspace` | local `path`, in-container `setup`, `plugin_paths` under `/user_ws` |
| `plugins` | target, gimbal, timing, sensor and world specifications |

Each plugin specification contains `entry: "python.module:ClassName"` and
`config: {…}`. Defaults are `p1sim_dev.builtins:{Trajectory,KinematicGimbal,Timing,Sensor,World}`.
The manifest records actual loaded module SHA-256, entry, version, API version,
configuration and declared observables. Local workspace references never copy
private source/weights into the configuration, image or Git repository.

Every invocation has a unique `run_id`. Identical config/seed has an identical
`run_key`, not the same process-instance identifier. Reset keeps run_id/config/
seed, increments `epoch`, and reconstructs all plugins with the same seed.
Per-kind RNG streams are independent; adding a sensor model cannot alter the
target RNG stream. Repeated analytic target evaluations at identical simulation
times are deterministic. Gazebo callback/service scheduling, image rendering,
wall time and real algorithm output are not promised bitwise deterministic.

## Reset and lifecycle

`./p1sim dev reset` calls `POST /reset/prepare` before restarting the simulator.
The request persists RESETTING and the next epoch, emits a lifecycle witness,
stops the old runtime (including recorder, pose services, plugins, Gazebo and
bridge), then starts it again. `/health` returns 503 until fresh clock/image/
CameraInfo callbacks and live Gazebo/bridge establish READY. Reset start,
STARTING, READY and ENDED appear in `lifecycle.jsonl`; phase snapshots are
atomic in `manifest.json`. Restarted Gazebo clock starts at zero. Restart
algorithms across reset to clear their time-dependent buffers. No arbitrary
HTTP reset is implied by a health probe. This API performs reset preparation;
the public CLI owns the supervised restart and readiness wait.

Bags have an explicit epoch through `/p1sim/state`; each recorder ends before
reset. `dev record` after reset records the new epoch. READY means the simulation
is producing fresh sensors, not that an attached algorithm has acquired a target.

## Built-in platform models

`Trajectory` selects `mode: static | linear | accel | reverse`. Coordinates
are Gazebo world meters. `velocity_m_s`, `acceleration_m_s2`, `offset_m` are
three-element vectors; optional `seed_offset_m` draws a seeded origin offset.
`reverse_period_seconds` is each leg's duration. Reverse is continuous piecewise
linear motion, with instantaneous velocity reversals. All target models apply
to authored target initial poses; extension callbacks may supply per-target
behavior based on that pose. No anti-spin or multi-armor evaluator is implied.
The runtime moves the actual Gazebo target model through its pose service.
Requested trajectory (with `source_time_ns`) and last service-confirmed presentation are distinct
observables. Pose updates are asynchronous and limited to one service in flight;
there is no claim of exact image/target/actuator lockstep synchronization.

`KinematicGimbal` defaults to absolute-angle compatibility. Optional
`rate_limit_rad_s`, `acceleration_limit_rad_s2`, two-axis `saturation_rad`,
and `application_delay_seconds` configure a simulation-time model. Pending
commands are ordered and coalesced at model steps. The model's q/v describe
its requested trajectory; JointState q describes pose-service confirmations and v is the finite difference
between successive clock-stamped JointState samples (zero while stationary).
Applied-event v is the interval average between pose-service confirmations. Applied velocity is a finite difference of confirmed angles
over simulator time and is not an inertial motor measurement. Bounds constrain
the selected model, not Gazebo service wall latency. The orientation-only IMU
retains its unavailable angular velocity/acceleration covariance convention.

`Timing` adds `delay_seconds` plus seeded uniform `jitter_seconds` (0 by default,
maximum total 5 s). Images and CameraInfo keep their source stamp and order;
`delivery_time_ns` identifies their later publication. The queue is bounded at
256 frames and overflow fails the run. Large delays require downstream TF
buffers covering the delay. Sensor plugins can mutate pixels, drop/stale frames,
change calibration or report visibility at the publication boundary.
`examples.visibility_plugin:PeriodicVisibility` is an independently loaded
reference with periodic blackout, seeded drop probability and calibration scale.
A blackout changes the delivered raster; geometric FOV and optical occlusion
are separate observable facts.

## Plugin API 1

Subclass `p1sim_dev.plugins.Plugin`, declare `kind`, `api_version = 1`, semantic
`version` and tuple `observables`. No central runtime edits are required.

- `configure(config)` validates all options and fails before Gazebo starts.
- `start(Context(seed, epoch, kind))` initializes resources; default calls reset.
- `reset(context)` restores deterministic state; `context.rng()` supplies an independent seeded RNG.
- `close()` releases resources; must be safe after partially completed configure/start.

Callbacks use simulator seconds; lifecycle identity and epoch are supplied by
context. Required callbacks by kind:

| Kind | Callback / return |
|---|---|
| target | `state(seconds, initial_pose)` → position_m, rpy_rad, velocity_m_s, developer_truth |
| gimbal | `submit(q_rad, simulation_ns, sequence)`; `step(seconds)` → q_rad, v_rad_s, receipt_time_ns, command_sequence |
| timing | `delay(source_seconds)` → nonnegative finite delay seconds |
| sensor | `process(source_seconds, Image)` → image, dropped, occluded, calibration_scale |
| world | `resolve(scene, product_root)` → SDF path |

A world must supply the normal camera sensor/bridge topic and model, the
pose service, and developer target plate geometry if it uses the supplied
projection diagnostic. A physical motor plugin can implement the gimbal
callbacks without changing config/API. Plugins execute trusted Python code in
the runtime container. Put external plugins in the mounted workspace and list
their import directories under `workspace.plugin_paths`; every actual entry is
imported, checked for compatible kind/API and recorded with its file hash.
The API does not sandbox untrusted plugin code or discover arbitrary packages.

## Versioned observables and compatibility

All existing JSON String fields remain. Added envelopes use `schema` and
`schema_version: 1`, plus run_id/run_key, foundation/interface/plugin versions,
config_digest, seed, epoch and exact source/release identity. Standard ROS
messages retain their original types, units and QoS. `/joint_states.velocity`
is populated with successive published-state finite differences. New reliable
String topics are `/p1sim/state`, `/p1sim/target_state`, `/p1sim/camera_timing`.
`/p1sim/state` contains lifecycle, plugin identities, target requested/applied
state, gimbal model/applied state and sensor effects. It publishes periodically
so bags begun after READY still acquire identity. Existing receipt/application/
visibility topics gain the same identity. Platform envelopes remain outside
algorithm sensor/command ABI. Status exposes the envelope under `foundation`.

Consumers must ignore unknown additive keys and match schema major version.
Adding optional fields, plugins or profiles is compatible. Removing/renaming
fields, changing units, frame/time/QoS meanings, required fields, default
kinematics or CLI behavior requires Foundation V2 and a new interface/config
major where affected. A plugin callback signature change requires plugin API
V2. V1 has no automatic config migration; future migration must be explicit,
produce a new canonical digest and preserve the original. Deprecations require
release notes and a supported migration path across at least one minor release;
no existing V1 surface is removed in this release candidate.
