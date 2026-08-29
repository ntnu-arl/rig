#pragma once

// C++
#include <map>
#include <memory>
#include <mutex>
#include <vector>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// RIG
#include "rig/common/barometry.hpp"
#include "rig/common/config.hpp"
#include "rig/common/utilities.hpp"
#include "rig/graph/factor/CombinedImuFactor2.h"
#include "rig/graph/factor/DifferentialBarometryFactor.h"
#include "rig/graph/factor/DopplerFactor.h"
#include "rig/graph/factor/VelocityFactor.h"
#include "rig/graph/state.hpp"
#include "rig/imu/imu_manager.hpp"
#include "rig_msgs/GraphDebug.h"

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
// TF
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

using gtsam::symbol_shorthand::A;  // barometer (altitude) bias
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::E;  // radar-IMU extrinsics pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (R,t) if USE_POS, otherwise Rot3

namespace rig
{
void declare_config(ImuPreintegrationConfig & cfg);
void declare_config(RelinearizeThresholdConfig & cfg);
void declare_config(InitialSigmaConfig & cfg);
void declare_config(SmootherConfig & cfg);
void declare_config(GraphConfig & cfg);

class GraphManager
{
public:
#if USE_POS
  typedef gtsam::CombinedImuFactor ImuFactor;
  typedef gtsam::PreintegratedCombinedMeasurements PreintegratedMeasurements;
#else
  typedef gtsam::CombinedImuFactor2 ImuFactor;
  typedef gtsam::PreintegratedCombinedMeasurements2 PreintegratedMeasurements;
#endif

private:
  // Graph
  State current_state_;
  gtsam::Values current_values_;
  gtsam::NonlinearFactorGraph new_factors_;
  bool is_graph_initialized_;
  gtsam::Key current_graph_key_;
  gtsam::Key first_graph_key_;

  // Imu preintegration
  std::shared_ptr<gtsam::imuBias::ConstantBias> imu_bias_prior_;
  std::shared_ptr<PreintegratedMeasurements> imu_preintegrator_;
  boost::shared_ptr<PreintegratedMeasurements::Params> imu_preintegrator_params_;

  // logger
  std::unique_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_;

  std::unique_ptr<spdlog::logger> odom_logger_;

  // iSAM2 Params
  gtsam::ISAM2Params isam2_params_;
  gtsam::ISAM2 isam2_;

  // iFL Params
  gtsam::IncrementalFixedLagSmoother
    ifl_;  // Since iFL internally uses iSAM2, and we are only using one of them at a time, the isam2_params_ object gets reused as well as the additional iterations
  gtsam::FixedLagSmoother::KeyTimestampMap new_key_ts_map_;

  // Parameters
  GraphConfig config_;

  // Sensor Managers - only imu manager is required here
  std::shared_ptr<ImuManager> imu_mgr_ptr_;

  // Publishers
  ros::Publisher pub_odom_;
  ros::Publisher pub_odom_path_;
  ros::Publisher pub_pose_stamped_;
  ros::Publisher pub_baro_bias_;
  ros::Publisher pub_graph_debug_;
  ros::Publisher pub_extrinsic_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  geometry_msgs::TransformStamped tf_stamped_;
  geometry_msgs::PoseStamped pose_stamped_;
  nav_msgs::Path path_;

  void updateImuPreintegrator(const ImuMap & imu_measurements);

public:
  GraphManager(ros::NodeHandle & pnh, std::shared_ptr<ImuManager> imu_mgr_ptr);
  ~GraphManager();

  bool isGraphInitialized();

  void initializeGraph(
    const double ts, const gtsam::Key start_key, const gtsam::Pose3 & init_pose,
    const gtsam::Pose3 & init_extrinsic, const double & init_altitude = 0);

  void initializeImuPreintegrator(
    double gravity_magnitude, const Eigen::Matrix4d & T_B_I,
    const gtsam::Vector3 & gyro_bias_estimate = gtsam::Vector3::Zero());

#if USE_POS
  gtsam::CombinedImuFactor
#else
  gtsam::CombinedImuFactor2
#endif
  addImuFactor(const gtsam::Key old_key, const gtsam::Key new_key, const ImuMap & imu_measurements);

  void addFactors(const gtsam::NonlinearFactorGraph & new_factors);

  void addBarometerFactors(
    const gtsam::Key & prev_key, const gtsam::Key & key, const double & barometric_pressure,
    const double & sigma_height, const double & sigma_bias,
    const gtsam::noiseModel::mEstimator::Base::shared_ptr & robust);

  void addDopplerFactor(
    const gtsam::Point3 & point, const double & doppler, const gtsam::Pose3 & pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key & key, const double & sigma,
    const double & huber_threshold);

  void getImuPropagatedEstimate(
    gtsam::Values & estimate, const gtsam::Key key, gtsam::Pose3 & pose);

  void runUpdateStep(const gtsam::Values & initial_estimate = gtsam::Values());

  gtsam::Key getNewStateKey();

  void clearNewFactors();

  void clearNewKeyTsMap();
  void addValuesToKeyTsMap(const gtsam::Values & values, const double ts);
  void printIFLKeyTsMap();

  void getCurrentValues(gtsam::Values & values);

  void updateCurrentState(
    const gtsam::Key key, const double ts, const gtsam::Point3 & position = gtsam::Point3::Zero());

  gtsam::Matrix getCovariance(const gtsam::Key key);

  void publishResults(
    const double ts, const gtsam::Matrix & pose_covar, const gtsam::Matrix & vel_covar);

  void resetStateKeyIndex();
  double getCurrentTimestamp() const;

  const State & getCurrentState() const;

  gtsam::Key getCurrentKey() const;
  gtsam::Key getFirstKey() const;
};
}  // namespace rig
