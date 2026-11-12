# Third-party notices

This repository distributes project-authored Python, configuration, launch code
and geometric scene descriptions. It does not redistribute private algorithms,
weights, c37 binaries, competition robot meshes, or task qualification assets.

The Dockerfile installs upstream packages under their own licenses:

- ROS 2 / ros_gz / tf2 / robot_state_publisher: see the package manifests and
  copyright files in `/opt/ros/humble/share` (principally Apache-2.0/BSD).
- Gazebo Fortress / Ignition: Apache-2.0 components and dependencies.
- Ubuntu, Mesa, Xvfb, Python: see `/usr/share/doc/*/copyright` in the image.
- Docker official ROS image: https://hub.docker.com/_/ros

These are package-manager dependencies, not vendored source. Container images
are built locally; no private locally installed image is required or published.
