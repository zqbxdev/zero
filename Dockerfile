FROM ubuntu:jammy-20260210.1 AS base
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# 中文：base 阶段用于生产和真船运行，只安装最小 ROS2 运行环境。
# English: The base stage is for production and real-vessel runtime with only the minimal ROS2 runtime.
# ---------------------------------------------------------
RUN sed -i 's/[a-z\.]*archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/https/http/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common && \
    add-apt-repository universe -y && \
    sed -i 's/http/https/g' /etc/apt/sources.list && \
    locale-gen zh_CN zh_CN.UTF-8 && \
    update-locale LANG=zh_CN.UTF-8 LC_ALL=zh_CN.UTF-8

# 中文：设置默认语言环境，避免 ROS2 工具出现 locale 警告。
# English: Set the default locale to avoid locale warnings from ROS2 tools.
ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8

# ---------------------------------------------------------
# 中文：注册 ROS2 Humble 软件源，并使用清华镜像加速 ROS2 包下载。
# English: Register the ROS2 Humble repository and use the Tsinghua mirror to speed up ROS2 package downloads.
# ---------------------------------------------------------
RUN curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
> /etc/apt/sources.list.d/ros2.list

# ---------------------------------------------------------
# 中文：安装最小 ROS2 运行时和 CycloneDDS，适合生产部署与真船运行。
# English: Install minimal ROS2 runtime and CycloneDDS for production deployment and real-vessel operation.
# ---------------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 中文：让交互式 shell 自动加载 ROS2 Humble 环境。
# English: Make interactive shells automatically source the ROS2 Humble environment.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 中文：设置默认工作目录，供 Compose 挂载项目源码和 ROS2 工作区使用。
# English: Set the default working directory for Compose-mounted project source and ROS2 workspace.
WORKDIR /workspace

FROM base AS dev

# ---------------------------------------------------------
# 中文：dev 阶段用于远程编码和日常编译，增加开发工具但不安装 Gazebo/RViz 大型 GUI 组件。
# English: The dev stage is for remote coding and daily builds, adding development tools without heavy Gazebo/RViz GUI components.
# ---------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    git \
    tmux \
    wget \
    nano \
    file \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-rmw-fastrtps-cpp && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

FROM dev AS full

# ---------------------------------------------------------
# 中文：full 阶段用于实体机仿真、Gazebo/RViz 和完整 GUI 调试。
# English: The full stage is for physical-machine simulation, Gazebo/RViz, and complete GUI debugging.
# ---------------------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/gazebo-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    gz-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 中文：配置 GUI/Qt 运行参数，支持通过 Docker Compose 转发 Gazebo/RViz 图形程序。
# English: Configure GUI/Qt runtime variables for forwarding Gazebo/RViz graphical applications through Docker Compose.
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
