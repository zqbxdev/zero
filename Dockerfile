FROM ubuntu:jammy-20260210.1
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# 中文：基础环境阶段，切换 Ubuntu 软件源并安装证书、下载、GPG、发行版识别和语言环境工具。
# English: Base environment stage; switch Ubuntu package mirrors and install certificate, download, GPG, release-detection, and locale tools.
# ---------------------------------------------------------
RUN sed -i 's/[a-z\.]*archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/https/http/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg2 lsb-release locales && \
    sed -i 's/http/https/g' /etc/apt/sources.list && \
    locale-gen zh_CN zh_CN.UTF-8 && \
    update-locale LANG=zh_CN.UTF-8 LC_ALL=zh_CN.UTF-8

# 中文：设置容器默认语言环境为中文 UTF-8，避免 ROS2/Gazebo 工具出现 locale 警告。
# English: Set the default container locale to Chinese UTF-8 to avoid locale warnings from ROS2/Gazebo tools.
ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8

# ---------------------------------------------------------
# 中文：安装通用开发工具，并启用 Ubuntu universe 仓库以满足 ROS2/Gazebo 依赖。
# English: Install common development tools and enable the Ubuntu universe repository for ROS2/Gazebo dependencies.
# ---------------------------------------------------------
RUN apt-get install -y --no-install-recommends \
    sudo software-properties-common tmux wget build-essential git nano file && \
    add-apt-repository universe -y

# ---------------------------------------------------------
# 中文：添加 ROS2 Humble 软件源密钥，使用清华镜像加速 ROS2 包下载。
# English: Add the ROS2 Humble repository key and use the Tsinghua mirror to speed up ROS2 package downloads.
# ---------------------------------------------------------
RUN curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# 中文：注册 ROS2 Humble apt 源，使用当前 Ubuntu 发行版代号自动匹配仓库路径。
# English: Register the ROS2 Humble apt repository and automatically match the repository path using the current Ubuntu codename.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
> /etc/apt/sources.list.d/ros2.list

# ---------------------------------------------------------
# 中文：添加 Gazebo Fortress 官方稳定源，用于安装 LTS 版本 Gazebo 和 GUI 组件。
# English: Add the official Gazebo Fortress stable repository for the LTS Gazebo release and GUI components.
# ---------------------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/gazebo-archive-keyring.gpg

# 中文：注册 Gazebo stable apt 源，使用当前 Ubuntu 发行版代号选择兼容的软件包。
# English: Register the Gazebo stable apt repository and use the current Ubuntu codename to select compatible packages.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/gazebo-stable.list

# ---------------------------------------------------------
# 中文：安装 ROS2 Humble 桌面版、开发工具、Gazebo Fortress、ROS-Gazebo 桥接包和常用 RMW 实现。
# English: Install ROS2 Humble desktop, development tools, Gazebo Fortress, ROS-Gazebo bridge packages, and common RMW implementations.
# ---------------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    gz-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 中文：让 root 用户的交互式 shell 自动加载 ROS2 Humble 环境。
# English: Make the root user's interactive shell automatically source the ROS2 Humble environment.
# ---------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 中文：配置 GUI/Qt 运行参数，支持通过 Docker Compose 转发 Gazebo/RViz 等图形程序。
# English: Configure GUI/Qt runtime variables for forwarding graphical applications such as Gazebo/RViz through Docker Compose.
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# 中文：设置默认工作目录，供 Compose 挂载 ROS2 工作区和开发代码使用。
# English: Set the default working directory for Compose-mounted ROS2 workspaces and development code.
WORKDIR /workspace
