#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>

#include <yaml-cpp/yaml.h>

#include <angles/angles.h>

namespace YAML
{
template <>
struct convert<Eigen::Quaterniond>
{
  static Node encode(const Eigen::Quaterniond & rhs)
  {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    node.push_back(rhs.w());
    return node;
  }

  static bool decode(const Node & node, Eigen::Quaterniond & rhs)
  {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs = Eigen::Quaterniond(
      node[3].as<double>(), node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    return true;
  }
};
}  // namespace YAML

namespace rig
{
struct GravityConfig
{
  bool aligned_initialization = true;
  bool estimate = false;
  double magnitude = 9.81;
};

// IMU
struct ImuPreintegrationConfig
{
  double accel_noise_density = 0.0013886655606357616;
  double accel_bias_random_walk = 8.538212723310593e-05;
  double gyro_noise_density = 5.4380545102010436e-05;
  double gyro_bias_random_walk = 1.6587925152480572e-06;
  double integration_covariance = 0.0;
  double bias_acc_omega_int = 0.0;
};
struct ImuConfig
{
  std::string log_directory = "/tmp/rig";
  int log_level = 2;  // TODO: make string
  double pose_init_wait = 1.0;
  double ts_offset = 0.0;
  double interpolation_max_ts_diff = 0.1;
  double extrapolation_max_ts_diff = 0.1;
  bool estimate_gyro_bias = true;
};

// BAROMETER
struct BarometerConfig
{
  bool enabled = false;
  double sigma = 0.1;
  double bias_sigma = 1.0e-5;
  std::string robust_cost = "none";
};

// GRAPH
struct RelinearizeThresholdConfig
{
  double attitude_deg = 0.1;
  double attitude;
  double position = 0.1;
  double velocity = 0.1;
  double accel_bias = 0.1;
  double gyro_bias = 0.1;
  double extrinsic_attitude_deg = 0.1;
  double extrinsic_attitude;
  double extrinsic_position = 0.1;
};
struct InitialSigmaConfig
{
  Eigen::Vector3d attitude_deg = {0.1, 0.1, 0.1};
  Eigen::Vector3d attitude;
  Eigen::Vector3d position = {1e-6, 1e-6, 1e-6};
  Eigen::Vector3d velocity = {1e-3, 1e-3, 1e-3};
  Eigen::Vector3d accel_bias = {0.1, 0.1, 0.1};
  Eigen::Vector3d gyro_bias = {1e-5, 1e-5, 1e-5};
  Eigen::Vector3d extrinsic_attitude_deg = {1.0, 1.0, 1.0};
  Eigen::Vector3d extrinsic_attitude = {};
  Eigen::Vector3d extrinsic_position = {1.0e-2, 1.0e-2, 1.0e-2};
  double barometer_bias = 0.1;
};
struct SmootherConfig
{
  double lag = 1.0;

  InitialSigmaConfig initial_sigma;

  RelinearizeThresholdConfig relin_thresh;

  int relinearize_skip = 1;
  bool enable_relinearization = true;
  bool evaluate_nonlinear_error = false;
  std::string factorization_method = "CHOLESKY";
  bool cache_linearized_factors = true;
  bool enable_detailed_results = false;
  bool enable_partial_relinearization_check = false;
  bool find_unused_factor_slots = true;
};
struct GraphConfig
{
  std::string log_directory = "/tmp/rig";
  int log_level = 2;
  int additional_iterations = 1;
  std::string world_frame = "odom";
  std::string body_frame = "imu";
  bool use_fixed_lag = true;

  Eigen::Vector3d accel_bias_prior = {0, 0, 0};
  Eigen::Vector3d gyro_bias_prior = {0, 0, 0};

  SmootherConfig smoother;
  ImuPreintegrationConfig preint;

  BarometerConfig barometer;
};

// RADAR
struct RangeFilterConfig
{
  double min = 0.1;
  double max = 20.0;
};
struct RadarFilterConfig
{
  RangeFilterConfig range;
  double min_db = 5.0;
  double azimuth_threshold_deg = 60.0;
  double azimuth_threshold;
  double elevation_threshold_deg = 60.0;
  double elevation_threshold;
};
struct RadarSensorConfig
{
  Eigen::Vector3d l_BR_B = {0, 0, 0};       // position of {R} wrt {B} expressed in {B}
  Eigen::Quaterniond q_R_B = {0, 0, 0, 1};  // rotation from {R} to {B}

  double frame_time_ms;
  double frame_time;
  double range_sigma;
  double doppler_resolution;
  double doppler_sigma;
  double doppler_max;
  double phase_resolution_deg = 5.625;
  double phase_sigma;
};

struct DopplerRobustConfig
{
  std::string lesser = "fair";
  std::string greater = "welsch";
};
struct DopplerConfig
{
  DopplerRobustConfig robust_cost;
};

struct PointConfig
{
  bool enabled = false;
  std::string robust_cost = "none";
};

struct MapConfig
{
  bool enabled = true;
  size_t n_scans_delay = 0;
  float voxel_grid_leaf_size = 0.1f;
  float search_radius = 0.5;
  int minimum_neighbors = 20;
};

struct RadarConfig
{
  std::string log_directory = "/tmp/rig";
  int log_level = 2;
  bool use_angular_velocity = true;
  bool exp_avg_angvel = false;
  bool angle_noise = true;

  RadarFilterConfig filter;
  GravityConfig gravity;  // TODO move to graph manager probably

  bool use_fixed_lag;  // TODO remove necessity

  RadarSensorConfig sensor;

  bool estimate_extrinsic = true;  // note: point factor always estimates extrinsic
  DopplerConfig doppler;
  PointConfig point;
  MapConfig map;

  BarometerConfig barometer;
};

// common configs
void declare_config(BarometerConfig & cfg);
}  // namespace rig
