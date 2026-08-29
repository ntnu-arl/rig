# rig: Radar-Inertial Factor Graph Estimator

[![arXiv](https://img.shields.io/badge/arXiv-2605.01773-b31b1b)](https://arxiv.org/abs/2605.01773)
[![DOI](https://img.shields.io/badge/DOI-10.1109/TFR.2026.3690584-blue)](https://doi.org/10.1109/TFR.2026.3690584)
[![YouTube](https://img.shields.io/badge/YouTube-nS2WgZldIv4-red)](https://youtu.be/nS2WgZldIv4)

This repository contains the implementation of the work [On the Characterization and Limits of 4D Radar for Aided Inertial Navigation](https://arxiv.org/abs/2605.01773) published in IEEE Transactions on Field Robotics ([doi](https://doi.org/10.1109/TFR.2026.3690584)).

The method consists of a factor graph-based radar-inertial odometry fusing radar Doppler, radar range, and barometric pressure measurements using the incremental fixed-lag smoother from [GTSAM](https://github.com/borglab/gtsam).

[![Video Title Screen](https://img.youtube.com/vi/nS2WgZldIv4/maxresdefault.jpg)](https://www.youtube.com/watch?v=nS2WgZldIv4)

## Build

**Dependencies:**

Assuming you have a `ros-noetic-desktop-full` installation, install additional dependencies by

```bash
# apt
sudo apt install libeigen3-dev libpcl-dev libspdlog-dev libyaml-cpp-dev libgoogle-glog-dev \
  ros-noetic-pcl-conversions ros-noetic-eigen-conversions

# gtsam
git clone https://github.com/borglab/gtsam.git -b release/4.2
cd gtsam && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_PYTHON=OFF ..
make -j$(nproc) && sudo make install

# from source
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/MIT-SPARK/config_utilities.git && cd config_utilities && git checkout f569658 && cd ..
git clone https://github.com/ntnu-arl/rig.git
```

and build by

```bash
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build rig
```

## Use

**Online:**

```bash
roslaunch rig rig_node.launch
```

**Offline:**

```bash
roslaunch rig rig_rosbag.launch bag_name:=/path/to/bag.bag
```

Sensor specific and more general method parameters are set in `rig_ros/config/radar.yaml` and `rig_ros/config/rig.yaml`.

## Data

The datasets used in the paper are available on [huggingface](https://huggingface.co/datasets/ntnu-arl/rig_dataset), along with additional information regarding the physical setup and sensor data.

## Reference

If you use any of this implementation or accompanying data in your research, please cite:

```bibtex
@ARTICLE{nissov2026radar,
  author  = {Nissov, Morten and Alexis, Kostas},
  journal = {IEEE Transactions on Field Robotics},
  title   = {On the Characterization and Limits of 4-D Radar for Aided Inertial Navigation},
  year    = {2026},
  volume  = {3},
  pages   = {694--723},
  doi     = {10.1109/TFR.2026.3690584}
}
```
