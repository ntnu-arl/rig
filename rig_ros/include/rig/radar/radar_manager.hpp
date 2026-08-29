#pragma once

#define PCL_NO_PRECOMPILE

// C++
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Core>

// GTSAM
#include <gtsam/nonlinear/ISAM2.h>
// IMU preintegrator
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

// RIG
#include "rig/common/config.hpp"
#include "rig/common/stopwatch.hpp"
#include "rig/common/utilities.hpp"
#include "rig/graph/factor/DopplerHessianFactor.h"
#include "rig/graph/factor/PointToPointFactor.h"
#include "rig/graph/graph_manager.hpp"
#include "rig/imu/imu_manager.hpp"
#include "rig/radar/distribution.hpp"
#include "rig/radar/mapper.hpp"
#include "rig/radar/point.hpp"
#include "rig/radar/target_data.hpp"
#include "rig_msgs/RadarDebug.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// spdlog
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace rig
{
void declare_config(RangeFilterConfig & cfg);
void declare_config(RadarFilterConfig & cfg);
void declare_config(GravityConfig & cfg);
void declare_config(RadarSensorConfig & cfg);
void declare_config(RadarConfig & cfg);

// TODO: implement reset function
class RadarManager
{
public:
  RadarManager(ros::NodeHandle & pnh);
  ~RadarManager();

  void baroCallback(const sensor_msgs::FluidPressureConstPtr & msg);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
  void imuCallback(const sensor_msgs::ImuConstPtr & msg);

private:
  void initializeLogger();
  void assignRobustCost(
    const std::string & param, gtsam::noiseModel::mEstimator::Base::shared_ptr & robust_cost_ptr,
    const std::string & text = "");

  void convertPointCloud(
    const sensor_msgs::PointCloud2 & msg, pcl::PointCloud<radar::mmWavePoint> & cloud);
  bool initialize(const double & ts, const gtsam::Key & key);
  bool processCloud(
    const sensor_msgs::PointCloud2 & cloud_msg, const double & ts, const gtsam::Key & key);
  void filterPointCloud(const sensor_msgs::PointCloud2 & cloud_msg, TargetVector & valid_targets);

  void publishDebugCloud(
    const std_msgs::Header & header, const TargetVector & targets, const ros::Publisher & pub);
  template <typename PointT>
  void publishCloud(
    const std_msgs::Header & header, const pcl::PointCloud<PointT> & cloud,
    const ros::Publisher & pub);
  void publishGaussians(const std_msgs::Header & header, const DistributionVector & gaussians);

  void resetIntegrator(
    const double ts, const gtsam::NavState & state, const gtsam::imuBias::ConstantBias & bias);

private:
  std::shared_ptr<Mapper> mapper_ptr_;

  std::shared_ptr<ImuManager> imu_mgr_ptr_;
  std::unique_ptr<GraphManager> graph_mgr_ptr_;

  gtsam::noiseModel::mEstimator::Base::shared_ptr doppler_lesser_robust_cost_ptr_;
  gtsam::noiseModel::mEstimator::Base::shared_ptr doppler_greater_robust_cost_ptr_;
  gtsam::noiseModel::mEstimator::Base::shared_ptr point_robust_cost_ptr_;
  gtsam::noiseModel::mEstimator::Base::shared_ptr barometer_robust_cost_ptr_;

  Eigen::Isometry3d T_B_I_{Eigen::Isometry3d::Identity()};  // assuming IMU == BODY

  // high rate output
  bool preint_init_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preint_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
  std::mutex preint_mutex_;
  gtsam::NavState hr_state_{};
  double prev_ts_{0};

  rig_msgs::RadarDebug debug_msg_;

  // Logging
  std::unique_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

  // Parameters
  RadarConfig config_;

  // Publishers
  ros::Publisher pub_radar_debug_;
  ros::Publisher pub_cloud_filter_;
  ros::Publisher pub_cloud_map_;
  ros::Publisher pub_cloud_gaussians_;
  ros::Publisher pub_cloud_lesser_;
  ros::Publisher pub_cloud_greater_;

  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_debug_twist_;
  // Subscribers
  ros::Subscriber sub_baro_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_cloud_;
};
}  // namespace rig
