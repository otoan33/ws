FROM osrf/ros:jazzy-desktop

# Gazeboのリポジトリを追加
RUN apt-get update && apt-get install -y wget lsb-release gnupg curl \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update

# システムツールとビルド環境のインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    # システムツール
    sudo \
    gosu \
    less \
    bash-completion \
    git \
    vim \
    wget \
    # ネットワークツール
    iputils-ping \
    net-tools \
    dnsutils \
    # ビルドツール
    build-essential \
    libpython3-dev \
    python3-pip \
    astyle \
    && rm -rf /var/lib/apt/lists/*

# ROS 2の基本パッケージのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-tf2-tools \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
    
# Gazebo Harmonicシミュレーション関連パッケージのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# コントローラー関連パッケージのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-effort-controllers \
    ros-jazzy-position-controllers \
    ros-jazzy-velocity-controllers \
    && rm -rf /var/lib/apt/lists/*

# sudoの設定
RUN echo ALL ALL=\(ALL:ALL\) NOPASSWD: ALL>> /etc/sudoers

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
