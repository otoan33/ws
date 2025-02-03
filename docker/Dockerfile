FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu less bash-completion astyle \
    build-essential libpython3-dev python3-pip \
    git vim iputils-ping net-tools dnsutils wget\
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-tf2-tools ros-humble-robot-state-publisher ros-humble-joint-state-publisher\
    ros-humble-joint-state-publisher-gui\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-diff-drive-controller \
    ros-humble-effort-controllers \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN echo ALL ALL=\(ALL:ALL\) NOPASSWD: ALL>> /etc/sudoers

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
