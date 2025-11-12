FROM ubuntu:22.04

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic utilities and dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    git \
    git-lfs \
    build-essential \
    cmake \
    python3-pip \
    wget \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop and Mesa software rendering libraries
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up git-lfs
RUN git lfs install

# Create workspace directory
WORKDIR /root/ws_fanuc

# Clone FANUC repositories
RUN mkdir -p /root/ws_fanuc/src && \
    cd /root/ws_fanuc/src && \
    git clone https://github.com/FANUC-CORPORATION/fanuc_description.git && \
    git clone --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git

# Patch fanuc_libs to use ament_cmake (eliminates warning message)
RUN sed -i '7 a find_package(ament_cmake REQUIRED)' /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "# Make this an ament package" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "ament_package()" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt

# Install FANUC dependencies
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y && \
    rm -rf /var/lib/apt/lists/*

# Build FANUC libraries
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1

# Set up environment sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_fanuc/install/setup.bash" >> /root/.bashrc

# Set the working directory
WORKDIR /root/ws_fanuc

# Default command
CMD ["/bin/bash"]

