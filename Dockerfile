FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# apt
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    git cmake build-essential \
    libeigen3-dev libpcl-dev libspdlog-dev \
    libyaml-cpp-dev libboost-all-dev libgoogle-glog-dev \
    ros-noetic-angles \
    ros-noetic-pcl-conversions \
    ros-noetic-eigen-conversions \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

# gtsam
WORKDIR /root
RUN git clone https://github.com/borglab/gtsam.git -b release/4.2 --depth 1
WORKDIR /root/gtsam/build
RUN cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_POSE3_EXPMAP=ON \
      -DGTSAM_ROT3_EXPMAP=ON \
      -DGTSAM_USE_QUATERNIONS=ON \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_BUILD_PYTHON=OFF \
      .. && \
    make -j$(nproc) && \
    make install

# ros related
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/MIT-SPARK/config_utilities.git && \
    cd config_utilities && git checkout f569658
COPY . rig/

WORKDIR /root/catkin_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build rig
