// RIG
#include "rig/imu/imu_manager.hpp"

namespace rig
{
void declare_config(ImuConfig & cfg)
{
  using namespace config;
  name("ImuConfig");
  field(cfg.log_directory, "log_directory");
  {
    NameSpace ns{"imu"};
    field(cfg.log_level, "log_level", "trace|debug|info|warn|error|critical");
    field(cfg.pose_init_wait, "pose_init_wait", "s");
    field(cfg.ts_offset, "ts_offset", "s");
    field(cfg.interpolation_max_ts_diff, "interpolation_max_ts_diff", "s");
    field(cfg.extrapolation_max_ts_diff, "extrapolation_max_ts_diff", "s");
    field(cfg.estimate_gyro_bias, "estimate_gyro_bias");
  }

  // TODO: verify params
}

ImuManager::ImuManager(ros::NodeHandle & pnh)
: config_(config::checkValid(config::fromRos<ImuConfig>(pnh)))
{
  // Reset IMU Buffer
  imu_buffer_.clear();

  // Create logging sinks
  logger_console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_console_sink_->set_level(spdlog::level::trace);
  logger_file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    config_.log_directory + "imu_manager.log", true);
  logger_file_sink_->set_level(spdlog::level::trace);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_unique<spdlog::logger>("imu_manager", sinks.begin(), sinks.end());
  logger_->set_level(static_cast<spdlog::level::level_enum>(config_.log_level));
  logger_->flush_on(static_cast<spdlog::level::level_enum>(config_.log_level));

  logger_->info("Initialized with params:\n {}", config::toString(config_));
}

// Destructor
ImuManager::~ImuManager() {}

void ImuManager::addToImuBuffer(
  double ts, double acc_x, double acc_y, double acc_z, double gyr_x, double gyr_y, double gyr_z)
{
  // Convert to gtsam type
  gtsam::Vector6 imu_meas;
  imu_meas << acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;

  // Add to buffer
  std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
  imu_buffer_[ts + config_.ts_offset] = imu_meas;
}

void ImuManager::addToBaroBuffer(double ts, double pressure)
{
  std::lock_guard<std::mutex> lock(baro_buffer_mutex_);
  baro_buffer_[ts] = pressure;
}

gtsam::Vector6 ImuManager::interpolateImuMeasurement(
  const double ts1, const gtsam::Vector6 & meas1, const double ts2, const double ts3,
  const gtsam::Vector6 & meas3)
{
  if (ts3 - ts1 > config_.interpolation_max_ts_diff) {
    std::string msg =
      "IMU interpolation between timestamps that are too far apart: " + std::to_string(ts3) + "/" +
      std::to_string(ts1) + " diff: " + std::to_string(ts3 - ts1) +
      " Max allowed: " + std::to_string(config_.interpolation_max_ts_diff);
    throw std::invalid_argument(msg);
  }
  double ts_diff_ratio = (ts2 - ts1) / (ts3 - ts1);  //(x2-x1)/(x3-x1)
  gtsam::Vector6 meas2 =
    (meas3 - meas1) * ts_diff_ratio + meas1;  // y2 = (y3-y1) * ((x2-x1)/(x3-x1)) + y1
  return meas2;
}

double ImuManager::interpolateBaroMeasurement(
  const double & ts1, const double & meas1, const double & ts2, const double & ts3,
  const double & meas3)
{
  if (ts3 - ts1 > config_.interpolation_max_ts_diff) {
    std::string msg =
      "Baro interpolation between timestamps that are too far apart: " + std::to_string(ts3) + "/" +
      std::to_string(ts1) + " diff: " + std::to_string(ts3 - ts1) +
      " Max allowed: " + std::to_string(config_.interpolation_max_ts_diff);
    throw std::invalid_argument(msg);
  }
  double ts_diff_ratio = (ts2 - ts1) / (ts3 - ts1);  //(x2-x1)/(x3-x1)
  const double meas2 =
    (meas3 - meas1) * ts_diff_ratio + meas1;  // y2 = (y3-y1) * ((x2-x1)/(x3-x1)) + y1
  return meas2;
}

gtsam::Vector6 ImuManager::extrapolateImuMeasurement(
  const double ts1, const gtsam::Vector6 & meas1, const double ts2, const gtsam::Vector6 & meas2,
  const double ts3)
{
  if (ts3 - ts2 > config_.extrapolation_max_ts_diff) {
    std::string msg = "IMU extrapolation for timestamp that is too far past ts2. ts_diff = " +
                      std::to_string(ts3 - ts2) +
                      " Max allowed: " + std::to_string(config_.extrapolation_max_ts_diff);
    throw std::invalid_argument(msg);
  }
  double ts_diff_ratio = (ts3 - ts1) / (ts2 - ts1);  //(x2-x1)/(x3-x1)
  gtsam::Vector6 meas3 =
    (meas2 - meas1) * ts_diff_ratio + meas1;  // y3 = (y2-y1) * ((x2-x1)/(x3-x1)) + y1
  return meas3;
}

void ImuManager::getImuBufferIteratorsInInterval(
  const double ts_start, const double ts_end, ImuMapItr & s_itr, ImuMapItr & e_itr)
{
  // Check if timestamps are in correct order
  if (ts_start >= ts_end) {
    std::string msg = "IMU lookup timestamps ts_start(" + std::to_string(ts_start) +
                      ") >= ts_end(" + std::to_string(ts_end) + ")";
    throw std::invalid_argument(msg);
  }

  logger_->debug(
    "Lookup timestamps: {}, {}. Buffer contains: {} {}", ts_start, ts_end,
    imu_buffer_.begin()->first, imu_buffer_.rbegin()->first);

  // Get Iterator Belonging to ts_start
  s_itr = imu_buffer_.lower_bound(ts_start);
  // Get Iterator Belonging to ts_end
  e_itr = imu_buffer_.lower_bound(ts_end);

  // Check if it is first value in the buffer which means there is no value before to interpolate
  // with
  if (s_itr == imu_buffer_.begin())
    logger_->warn(
      "IMU lookup requiring first message of the buffer s_itr == imu_buffer_.begin() --- Might be "
      "during IMU Pose Initialization");

  // Check if last value is valid
  if (e_itr == imu_buffer_.end()) {
    logger_->warn(
      "IMU lookup is past IMU buffer, with lookup start ts {} and end ts {} and latest IMU ts is "
      "{}",
      ts_start, ts_end, imu_buffer_.rbegin()->first);
    // Go backwards to get a valid iterator
    --e_itr;
  }

  // Check if two IMU messages are different
  if (s_itr == e_itr) {
    std::string msg =
      "Not Enough IMU measurements between timestamps, with Start/End Timestamps: " +
      std::to_string(ts_start) + "/" + std::to_string(ts_end) +
      ", with diff: " + std::to_string(ts_end - ts_start);
    throw std::length_error(msg);
  }
}

void ImuManager::getInterpolatedImuMeasurements(
  const double ts_start, const double ts_end, ImuMap & interpolated_imu_map)
{
  // clear
  interpolated_imu_map.clear();

  // Get nearest ImuMap iterators corresponding to timestamps
  ImuMapItr s_itr, e_itr;
  std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
  try {
    logger_->debug("Looking up timestamps: [{}, {}]", ts_start, ts_end);
    logger_->debug(
      "Buffer size: {}, first time: {}", imu_buffer_.size(), imu_buffer_.begin()->first);
    logger_->debug(
      "Buffer contains: {} {}", imu_buffer_.begin()->first, imu_buffer_.rbegin()->first);
    getImuBufferIteratorsInInterval(ts_start, ts_end, s_itr, e_itr);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
  // Copy in between IMU measurements
  interpolated_imu_map.insert(s_itr, e_itr);  // Note: Value at e_itr is not inserted

  // Interpolate first element
  if (s_itr->first > ts_start) {
    auto prev_s_itr = s_itr;
    --prev_s_itr;
    gtsam::Vector6 ts_start_meas = interpolateImuMeasurement(
      prev_s_itr->first, prev_s_itr->second, ts_start, s_itr->first, s_itr->second);

    // Add Interpolated Value to return ImuMap
    interpolated_imu_map[ts_start] = ts_start_meas;
  }

  // Add last element
  if (e_itr->first > ts_end) {
    // Interpolate IMU message at timestamp
    auto prev_e_itr = e_itr;
    --prev_e_itr;
    gtsam::Vector6 ts_end_meas = interpolateImuMeasurement(
      prev_e_itr->first, prev_e_itr->second, ts_end, e_itr->first, e_itr->second);

    interpolated_imu_map[ts_end] = ts_end_meas;
  } else {
    // Add last actual IMU message
    interpolated_imu_map[e_itr->first] =
      e_itr->second;  ////std::map insert doesn't insert last iterator so if e_itr->first >
                      ///ts_end then it means
                      /// we are at the end of IMU buffer and we need to insert the last IMU
                      /// message to the return buffer
    // Extrapolate IMU message at timestamp(ts_end>k), e_itr(k), prev_e_itr(k-1)
    auto prev_e_itr = e_itr;
    --prev_e_itr;
    gtsam::Vector6 ts_end_meas = extrapolateImuMeasurement(
      prev_e_itr->first, prev_e_itr->second, e_itr->first, e_itr->second, ts_end);
    // Add Extrapolated Value to return ImuMap
    interpolated_imu_map[ts_end] = ts_end_meas;
  }
}

void ImuManager::getInterpolatedBaroMeasurement(
  const double & ts, double & interpolated_baro_measurement)
{
  std::lock_guard<std::mutex> lock(baro_buffer_mutex_);

  // find first element with stamp >= ts
  BaroMapItr e_itr = baro_buffer_.lower_bound(ts);
  BaroMapItr s_itr = std::prev(e_itr, 1);

  // Check if it is first value in the buffer which means there is no value before to interpolate
  // with
  if (e_itr == baro_buffer_.begin())
    logger_->warn(
      "Baro lookup requiring first message of the buffer e_itr == baro_buffer_.begin()");

  // Check if last value is valid
  if (e_itr == baro_buffer_.end()) {
    logger_->warn(
      "Baro lookup is past Baro buffer, with lookup ts {} and latest Baro ts is "
      "{}",
      ts, baro_buffer_.rbegin()->first);
    // Go backwards to get a valid iterator
    --e_itr;
  }

  interpolated_baro_measurement =
    interpolateBaroMeasurement(s_itr->first, s_itr->second, ts, e_itr->first, e_itr->second);
}

void ImuManager::estimateAttitudeFromImu(
  const double imu_pose_init_ts, gtsam::Rot3 & init_attitude, double & gravity_magnitude,
  gtsam::Vector3 & gyro_bias)
{
  {
    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);

    const double imu_buffer_duration = imu_pose_init_ts - imu_buffer_.begin()->first;
    if (imu_buffer_duration < config_.pose_init_wait) {
      std::string msg =
        "Imu buffer not filled; current/required size: " + std::to_string(imu_buffer_duration) +
        "/" + std::to_string(config_.pose_init_wait);
      throw std::length_error(msg);
    }
  }
  // Get IMU measurments in the interval from config_.pose_init_wait before the imu_pose_init_ts to
  // imu_pose_init_ts
  ImuMap init_imu_map;
  double prev_ts = imu_pose_init_ts - config_.pose_init_wait;
  // REVIEW: why the fuck is it interpolated?
  getInterpolatedImuMeasurements(prev_ts, imu_pose_init_ts, init_imu_map);
  // Accumulate Acceleration part of IMU Messages
  size_t imu_pose_init_msg_count = 0;  // Counter for messages needed for initializing Pose from IMU
  Eigen::Vector3d imu_pose_init_acc_mean(0.0, 0.0, 0.0);   // mean lin acc measured
  Eigen::Vector3d imu_pose_init_gyro_mean(0.0, 0.0, 0.0);  // mean ang vel measured
  for (auto itr = init_imu_map.begin(); itr != init_imu_map.end(); ++itr) {
    imu_pose_init_acc_mean += itr->second.head<3>();
    imu_pose_init_gyro_mean += itr->second.tail<3>();
    ++imu_pose_init_msg_count;
  }
  // Average IMU measurements and set assumed gravity direction
  imu_pose_init_acc_mean /= (double)imu_pose_init_msg_count;
  imu_pose_init_gyro_mean /= (double)imu_pose_init_msg_count;
  std::cout << "average imu: " << imu_pose_init_acc_mean.transpose() << '\n';
  gravity_magnitude = imu_pose_init_acc_mean.norm();
  Eigen::Vector3d g_unit_vec(0.0, 0.0, 1.0);  // ROS convention
  // Normalize gravity vectors to remove the effect of gravity magnitude from place-to-place
  imu_pose_init_acc_mean.normalize();

  // Calculate robot initial orientation using gravity vector.
  init_attitude =
    gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(imu_pose_init_acc_mean, g_unit_vec));

  // Estimate gyro bias
  if (config_.estimate_gyro_bias) Convert(imu_pose_init_gyro_mean, gyro_bias);

  std::cout << "Gravity Magnitude: " << gravity_magnitude << std::endl;
  std::cout << "Mean IMU Acceleration Vector(x,y,z): " << imu_pose_init_acc_mean.transpose()
            << " - Gravity Unit Vector(x,y,z): " << g_unit_vec.transpose() << std::endl;
  std::cout << "Yaw/Pitch/Roll(deg): " << init_attitude.ypr().transpose() * Degrees(1) << std::endl;
  std::cout << "Gyro Bias (rad): " << gyro_bias.transpose() << std::endl;
}

void ImuManager::estimateBarometerBias(const double & baro_init_ts, double & init_altitude)
{
  // TODO: more robust error handling
  double init_pressure = 0.0;
  {
    std::lock_guard<std::mutex> lock(baro_buffer_mutex_);

    const double baro_buffer_duration = baro_init_ts - baro_buffer_.begin()->first;
    logger_->info(
      "Barometer buffer contains {} secs of messages and {} msgs", baro_buffer_duration,
      baro_buffer_.size());

    BaroMapItr s_itr = baro_buffer_.begin();
    BaroMapItr e_itr = baro_buffer_.lower_bound(baro_init_ts);

    if (e_itr == baro_buffer_.begin())
      logger_->warn(
        "[Initialize] Baro lookup requiring first message of the buffer e_itr == "
        "baro_buffer_.begin()");

    // Check if last value is valid
    if (e_itr == baro_buffer_.end()) {
      logger_->warn(
        "[Initialize] Baro lookup is past Baro buffer, with lookup ts {} and latest Baro ts is "
        "{}",
        baro_init_ts, baro_buffer_.rbegin()->first);
      // Go backwards to get a valid iterator
      --e_itr;
    }

    logger_->debug("Initializing with baro messages from {} to {}", s_itr->first, e_itr->first);
    const size_t num_measurements = std::distance(s_itr, e_itr);
    for (auto itr = s_itr; itr != e_itr; ++itr) {
      init_pressure += itr->second;
    }
    init_pressure /= num_measurements;
  }

  init_altitude = barometry::height_from_pressure(init_pressure);

  logger_->info("Barometer bias: {} [m]", init_altitude);
}

}  // namespace rig
