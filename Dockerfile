# Dockerfile
# Base: JetPack 5.1.2 (L4T r35.4.1) runtime with CUDA/cuDNN
FROM nvcr.io/nvidia/l4t-jetpack:r35.4.1

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Locale + basic deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales lsb-release gnupg2 curl ca-certificates \
    build-essential git wget cmake sudo python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Add ROS repo for ARM64 (Jetson)
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && curl -sSL "http://repo.ros2.org/repos.key" | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'

# Install ROS Noetic Desktop + tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    python3-catkin-tools python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep
RUN rosdep init && rosdep update || echo "rosdep already initialized"

# Source ROS in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc


# --- existing Dockerfile content above ---
# After ROS is installed and sourced

# Install Orbbec ROS1 driver (Femto Mega + Femto Bolt support)
WORKDIR /root/catkin_ws/src
RUN git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS1.git

RUN apt-get update && apt-get install -y --no-install-recommends libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
ros-noetic-image-transport-plugins ros-noetic-compressed-image-transport \
ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-noetic-diagnostic-updater ros-noetic-diagnostic-msgs \
libdw-dev \
libelf-dev


# -------------------------
# Build workspace
# -------------------------
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Auto-source your workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# -------------------------
# Add watchdog script for driver restart
# -------------------------
COPY run_driver.sh /root/run_driver.sh
RUN chmod +x /root/run_driver.sh

# -------------------------
# Healthcheck: monitor ROS topic
# -------------------------
HEALTHCHECK --interval=30s --timeout=10s --start-period=20s --retries=3 CMD \
    bash -c "source /opt/ros/noetic/setup.bash && \
            source /root/catkin_ws/devel/setup.bash && \
            rostopic list | grep -q /orbbec/camera/color/camera_info" || exit 1

# -------------------------
# Entrypoint
# -------------------------
ENTRYPOINT ["/root/run_driver.sh"]

# Default entrypoint: start bash with ROS sourced
# CMD ["bash", "-i"]

# Start Femto Bolt driver automatically
# CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && \
#                    source /root/catkin_ws/devel/setup.bash && \
#                    roslaunch orbbec_camera femto_bolt.launch"]

# CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && \
# source /root/catkin_ws/devel/setup.bash && \
# roslaunch orbbec_camera femto_mega.launch"]
