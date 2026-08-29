#pragma once

// C++
#include <map>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// ROS
#include <ros/ros.h>

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

// RIG
#include "rig/common/barometry.hpp"
#include "rig/common/config.hpp"
#include "rig/common/utilities.hpp"

namespace rig
{
typedef std::map<
  double, gtsam::Vector6, std::less<double>,
  Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>>
  ImuMap;
typedef ImuMap::iterator ImuMapItr;

typedef std::map<double, double, std::less<double>> BaroMap;
typedef BaroMap::iterator BaroMapItr;

void declare_config(ImuConfig & cfg);

class ImuManager
{
private:
  std::mutex imu_buffer_mutex_;  // Mutex for reading writing to the IMU buffer
  ImuMap imu_buffer_;            // IMU buffer
  std::mutex baro_buffer_mutex_;
  BaroMap baro_buffer_;

  std::unique_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

  // Parameters
  ImuConfig config_;

public:
  // Constructor
  ImuManager(ros::NodeHandle & pnh);

  // Destructor
  ~ImuManager();

  void addToImuBuffer(
    double ts, double acc_x, double acc_y, double acc_z, double gyr_x, double gyr_y, double gyr_z);

  void addToBaroBuffer(double ts, double pressure);

  gtsam::Vector6 interpolateImuMeasurement(
    const double ts1, const gtsam::Vector6 & meas1, const double ts2, const double ts3,
    const gtsam::Vector6 & meas3);

  double interpolateBaroMeasurement(
    const double & ts1, const double & meas1, const double & ts2, const double & ts3,
    const double & meas3);

  gtsam::Vector6 extrapolateImuMeasurement(
    const double ts1, const gtsam::Vector6 & meas1, const double ts2, const gtsam::Vector6 & meas2,
    const double ts3);

  void getImuBufferIteratorsInInterval(
    const double ts_start, const double ts_end, ImuMapItr & s_itr, ImuMapItr & e_itr);

  void getInterpolatedImuMeasurements(
    const double ts_start, const double ts_end, ImuMap & interpolated_imu_map);

  void getInterpolatedBaroMeasurement(const double & ts, double & interpolated_baro_measurement);

  void estimateAttitudeFromImu(
    const double imu_pose_init_ts, gtsam::Rot3 & init_attitude, double & gravity_magnitude,
    gtsam::Vector3 & gyro_bias);

  void estimateBarometerBias(const double & baro_init_ts, double & init_altitude);
};
}  // namespace rig
