# Foundation V1 release candidate

Proposed tag: `v1.0.0-rc.1`. Product release identity `1.0.0-rc.1`;
Foundation contract `1.0.0`; Developer interface, config and plugin API remain 1.
This document prepares release notes. It does not authorize merge, tag creation
or GitHub Release publication.

## Changes

- Canonical JSON run configuration, seeded plugins, unique instance run_id,
  repeatable configuration run_key, exact loaded runtime checksums and epoch.
- Public config-driven `dev up`, supervised reset with observable lifecycle,
  and isolated `dev batch` with machine-readable results and verified teardown.
- Analytic static/linear/constant-acceleration/reversing targets applied to the
  actual Gazebo models; requested source time and confirmed application time.
- Optional gimbal rate/acceleration/saturation/delay model; standard JointState
  q/v and separately identified command receipt/model/application observations.
- Independently loadable Python API 1 for target, gimbal, timing, sensor, world.
  Reference seeded timing jitter/delay and external blackout/drop/calibration
  sensor model; private workspace plugin imports and fail-fast API checks.
- Versioned state/target/camera timing envelopes, status, bag identity and
  compatibility/deprecation rules. Existing Developer commands/topics persist.
- External scheduler caller and compatibility/numerical/live ROS test examples.

## Provenance and adoption

Original outcome: reusable P1SIM platform product, usable through its native
CLI, ROS messages, HTTP status and Gazebo runtime. This is not an algorithm
research score or scientific Provider campaign.

Current accepted capability and starting source:
[P1SIM a6d551f](https://github.com/tianxiansheng166/P1SIM/commit/a6d551fd6df049c92e715bcdebf5c3516d7e344a).
Frozen consumer:
[PinyVision f380dde](https://github.com/tianxiansheng166/PinyVision/commit/f380dde98d880fcd57d5e8f24dc0a76643fb6909).
Detector/Tracker/Planner, production YAML and installed owner model are unchanged.
The consumer attaches through the public config's local workspace reference.

The Foundation PR targets main and includes the accepted history of
[Developer closed-loop PR #1](https://github.com/tianxiansheng166/P1SIM/pull/1)
and [sustained profile PR #2](https://github.com/tianxiansheng166/P1SIM/pull/2).
Those stacked PRs are preserved for review/evidence; Foundation consolidates
the product changes without rewriting their commits. The independent
[PinyVision external-camera PR #2](https://github.com/tianxiansheng166/PinyVision/pull/2)
retains its bringup-only scope. No consumer-side Foundation business logic.

There is no local AGENTS.md/V2.2 marker in this product checkout. The task is
routed under the supplied Task Card 17/17A policy and Commander-pinned
`rm_improve@a4f6bcfe937ab0b9c264a1bc63a897286620027c:AGENTS.md` overlay.
No policy was edited. Prior T229/T230 source, evidence and public interface
documentation were reused and adapted; no scientific source was requalified.

## Frozen gap reconciliation

| Starting gap | Delivery | Verification |
|---|---|---|
| Run/config/seed | config.py, host.py; runtime checksum verification | unit + repeat serial caller |
| Lifecycle/reset | foundation.py, CLI reset and supervised restart | live reset identity + lifecycle journal |
| Headless batch | public CLI batch, isolated project, outputs | serial caller + external-stop |
| Target control | built-in analytic model + real pose service | numerical reverse/seed; live static/linear/accel/reverse |
| Gimbal q/v and dynamics | model step/submit + confirmed JointState/application | numerical constraints + live ROS observer |
| Sensor/timing | plugin callbacks, source-stamped delayed paired sensors | actual raster blackout, bag timing and consumer proof |
| Stable observables | schema/versioned envelopes and periodic bag identity | deserialize real bags; compatibility consumer regressions |
| Plugin API | config module/class imports, lifecycle/context, hashed identity | external mounted timing/world; incompatible API rejection |
| Compatibility | compatibility.json, Foundation docs, unchanged existing interfaces | generic quickstart + real full-chain 5 s ×2 and 30 s |

## Reproduce

Build the public image with `docker build -t p1sim-foundation:1.0.0-rc.1 .`.
Run `python3 -m unittest discover -s tests -v`, then use task-private state,
project/domain/HTTP port environment variables to avoid another local run.
`python3 tests/acceptance.py <result.json>` exercises the generic workflow.
`python3 tests/foundation_acceptance.py` checks real dynamics, reset and bags.
`python3 examples/batch_caller.py --output <new-directory>` performs three
serial public batch calls. `P1SIM_MATRIX_OUTPUT=<new-directory> python3
tests/foundation_matrix.py` checks more models, external plugins and failures.

For the real consumer, prepare the compatible dependency image and authorized
read-only workspace described in AUTOAIM.md, set `P1SIM_IMAGE` accordingly,
and create a local copy of `configs/autoaim.json` with workspace.path set.
Set `P1SIM_CONFIG` to that config, `P1SIM_STATE_DIR` to a fresh directory,
then run `P1SIM_WINDOWS=5,5,30 python3 tests/sustained_acceptance.py`.
The test records actual messages and invokes the unchanged loop and bag
observers. A second config can select Timing delay/jitter to prove continued
graph use with a non-default platform model. Do not distribute the private
workspace, models, dependency image or owner runtime installation.

## Limits and next evolution

Developer primitives do not qualify physical motor dynamics, camera noise
statistics, ballistics, scientific truth/scoring, arbitrary detector robustness,
HIL or private model redistribution. Rich anti-spin/multi-armor worlds, motor
models and sensor effects can use the extension API; they do not block this
Foundation candidate. No known incompatible change to Developer V1 is needed.
Changing existing defaults, field/unit/frame/clock meanings or plugin callback
signatures would require the major-version process in compatibility.json.

## Observed candidate verification

[Compact evidence](foundation-observed.json) and [exact compressed data manifest](foundation-data-manifest.json)
record the tested source, dependencies, negative results and cleanup.

| Real default consumer | Max detection / tracking / control gap | Exact receipts |
|---|---|---|
| First clean 5 s window | 0.265 / 0.070 / 0.015 s | 2,362 / 2,362 |
| Second clean 5 s window | 0.265 / 0.070 / 0.020 s | 2,278 / 2,278 |
| 30 s window | 0.265 / 0.135 / 0.020 s | 11,429 / 11,429 |
| Non-default timing, 5 s | 0.200 / 0.070 / 0.025 s | 2,261 / 2,261 |

All windows passed the original 0.5 s maximum-outage gate, source-time TF,
exact receipt sequences, applied JointState/TF, visible rendered targets and
command-induced camera changes. Nine unit tests, public-image generic workflow,
actual dynamics/blackout/reset/bag checks, three serial batch calls, linear/accel
trajectory application, mounted external timing/world plugins, API rejection,
external-stop and interrupted-startup cleanup passed. Private consumer source,
parameters and model hashes match the accepted baseline.

An earlier 30 s run failed at a 0.53 s tracking gap. Its original window and
failure are retained. The default model was corrected to preserve exact
absolute-angle same-clock application, and JointState velocity now clears
while stationary. The later passing windows do not establish arbitrary-load
robustness or isolate a single cause for the earlier detector/tracker loss.

The 5/5/30 proof uses runtime commit `7c8453e`; subsequent commit `6d488a5`
changes only host-side batch signal/process-group cleanup and its matrix test.
All ROS/Gazebo module hashes match. That host change and the non-default
consumer were verified on `6d488a5`. Later commits contain delivery evidence
only. The public Dockerfile build used the supported mirror arguments after
the default Ubuntu archive download stalled; no dependencies were piped into
a shell or privately installed consumer assets included in the public image.

Task containers and networks are absent. Twenty-two task-owned bags were
compressed and their decompressed SHA-256 verified before removing the DB3
copies, reclaiming 3,217,133,568 allocated bytes. Restore with `gzip -dk` before
ROS bag access. Current candidate images and the clean task checkout remain;
foreign/active Lane assets were untouched. Formal merge/tag/release is the
remaining adoption gate.
