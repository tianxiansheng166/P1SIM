# Foundation 1.0 — public developer-source preview

## Preview 2 maintenance (first-run UX)

- `dev doctor` now checks the selected local Docker image before starting an
  inspection container. It immediately reports the required `dev bootstrap`
  command if the image has not been built; no hidden Docker Hub pull or image
  build is initiated by `doctor`.
- Successful diagnostics force `--pull never`, preserving the explicit
  source-self-build model. Regression tests cover missing and installed images.
- The existing Foundation API/config/schema versions remain at 1.0.0/1.
- An isolated public-archive `bootstrap` attempt hit a local wall-time guard
  while downloading Ubuntu apt metadata; this is a `BOOTSTRAP_UNVERIFIED`
  outcome, not a successful fresh-host installation. See optional apt mirrors
  in the Chinese quickstart. Second-machine acceptance remains open.

## Included

- Local ROS 2 Humble / Gazebo Fortress CPU-headless environment and CLI.
- Versioned Foundation V1 config, seed, run identity, lifecycle/reset/epoch.
- Generic scene and one project-authored single-target AutoAim presentation.
- Image / CameraInfo / TF / IMU / JointState, target and gimbal observables.
- Simple kinematic gimbal, timing and visibility primitives; documented plugin seams.
- Bounded headless batch and retained platform execution receipts.
- English quickstart and Chinese first-user/install/AutoAim guides.

## Not included

- Private PinyVision source, proprietary model weights, competition meshes, frozen private reference binaries or SSH credentials.
- Private internal task logs, negative-run evidence, local workspace paths, internal asset hashes or approval screenshots.
- Scientific Fire/scorer truth, calibrated motor/ballistics, experimental TestSpec evaluation, hardware interface or race-robot control.
- A public GHCR image: source distribution builds locally with upstream ROS/Ubuntu packages.

## Publication and compatibility

The Foundation API and config versions are 1. No generic auto-updater or official
binary image is shipped. Run `sha256sum -c SHA256SUMS` before your first
bootstrap, and preserve the exact Git revision plus local Docker image digest
for reproducible use.

This source package is an **early public developer preview** until an independent
fresh-host Linux/Docker download-to-robot-workspace test is accepted. Do not
interpret it as experimental physics qualification or official hardware support.
