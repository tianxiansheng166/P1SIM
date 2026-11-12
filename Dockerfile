FROM ros:humble-ros-base@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu
ARG ROS_MIRROR=http://packages.ros.org/ros2/ubuntu
RUN sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" /etc/apt/sources.list \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_MIRROR}|g; s|Types: deb deb-src|Types: deb|" /etc/apt/sources.list.d/ros2.sources \
    && apt-get -o Acquire::Retries=3 update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-tf2-ros-py \
    ros-humble-tf2-geometry-msgs ros-humble-rosbag2-storage-default-plugins \
    ros-humble-ros2bag ros-humble-demo-nodes-cpp ros-humble-robot-state-publisher \
    python3-yaml libgl1-mesa-dri procps \
    && rm -rf /var/lib/apt/lists/*
# The pinned base can contain older rosbag2 Python/C++ bindings. Upgrade the
# entire installed rosbag2 family together with the already selected ros2bag CLI.
RUN apt-get -o Acquire::Retries=3 update && apt-get install -y --only-upgrade 'ros-humble-rosbag2*' \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/p1sim
COPY . .
RUN chmod -R a+rX /opt/p1sim
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "-m", "p1sim_dev.runtime"]
