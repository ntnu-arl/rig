cmake_minimum_required(VERSION 3.0.2)
project(rig)

# set(USE_POS false STRING "Include position variable in factor graph")
add_definitions(-DUSE_POS=true)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic-errors")
# Werror
# -Wconversion -Wsign-conversion

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  pcl_conversions
  eigen_conversions
  tf2
  tf2_ros
  tf2_eigen
  nav_msgs
  rig_msgs
  visualization_msgs
  rosbag
)

set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../gtsam/")
find_package(GTSAM 4.2.0 REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
message("GTSAM version: ${GTSAM_VERSION}")
message("GTSAM path: ${GTSAM_DIR}")
message("GTSAM path: ${GTSAM_UNSTABLE_DIR}")

find_package(PCL REQUIRED)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(config_utilities REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions eigen_conversions tf2 tf2_ros tf2_eigen nav_msgs rig_msgs visualization_msgs
  DEPENDS PCL EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

list(APPEND libs ${catkin_LIBRARIES} gtsam gtsam_unstable ${PCL_LIBRARIES} Eigen3::Eigen spdlog::spdlog yaml-cpp config_utilities::config_utilities)

# Libraries for individual components
add_library(estimator_backend
  src/radar/radar_manager.cpp
  src/imu/imu_manager.cpp
  src/graph/graph_manager.cpp
  src/graph/CombinedImuFactor2.cpp
  src/radar/mapper.cpp
  src/common/config.cpp
)
target_link_libraries(estimator_backend ${libs})

# # Nodes
add_executable(rig_node
  src/rig_node.cpp
)
target_link_libraries(rig_node estimator_backend ${libs})

add_executable(rig_rosbag
  src/rig_rosbag.cpp
)
target_link_libraries(rig_rosbag estimator_backend ${libs})
