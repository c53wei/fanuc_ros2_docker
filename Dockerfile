FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    less \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Installing and configuring git-lfs" && \
    apt-get update && \
    apt-get install -y git-lfs && \
    git lfs install && \
    rm -rf /var/lib/apt/lists/*

RUN echo "Checking out GitHub repositories" && \
    mkdir -p ~/ws_fanuc/src

WORKDIR /root/ws_fanuc/src

RUN git clone https://github.com/FANUC-CORPORATION/fanuc_description.git && \
    git clone --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git

# Patch fanuc_libs to use ament_cmake (eliminates warning message)
RUN sed -i '7 a find_package(ament_cmake REQUIRED)' /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "# Make this an ament package" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt && \
    echo "ament_package()" >> /root/ws_fanuc/src/fanuc_driver/fanuc_libs/CMakeLists.txt

RUN echo "Installing FANUC dependencies"
WORKDIR /root/ws_fanuc
RUN apt-get update && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y && \
    rm -rf /var/lib/apt/lists/*

RUN echo "Building FANUC libraries" && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=1 -DBUILD_EXAMPLES=1

RUN echo "Installing development dependencies" && \
    apt-get update && \
    apt-get install ros-humble-pymoveit2 && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash \nsource ~/ws_fanuc/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh"]

CMD ["/bin/bash"]
