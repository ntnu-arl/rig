#include "rig/graph/graph_manager.hpp"

namespace rig
{
void declare_config(RelinearizeThresholdConfig & cfg)
{
  using namespace config;
  name("RelinearizeThresholdConfig");
  field(cfg.attitude_deg, "attitude_deg", "deg");
  cfg.attitude = angles::from_degrees(cfg.attitude_deg);
  field(cfg.position, "position", "m");
  field(cfg.velocity, "velocity", "m/s");
  field(cfg.accel_bias, "accel_bias", "m/s^2");
  field(cfg.gyro_bias, "gyro_bias", "rad/s");
  field(cfg.extrinsic_attitude_deg, "extrinsic_attitude_deg", "deg");
  cfg.extrinsic_attitude = angles::from_degrees(cfg.extrinsic_attitude_deg);
  field(cfg.extrinsic_position, "extrinsic_position", "m");
}

void declare_config(InitialSigmaConfig & cfg)
{
  using namespace config;
  name("InitialSigmaConfig");
  field(cfg.attitude_deg, "attitude_deg", "deg[3]");
  for (int i = 0; i < 3; ++i) {
    cfg.attitude(i) = angles::from_degrees(cfg.attitude_deg(i));
  }
  field(cfg.position, "position", "m[3]");
  field(cfg.velocity, "velocity", "m/s[3]");
  field(cfg.accel_bias, "accel_bias", "m/s^2[3]");
  field(cfg.gyro_bias, "gyro_bias", "rad/s[3]");
  field(cfg.extrinsic_attitude_deg, "extrinsic_attitude_deg", "deg[3]");
  for (int i = 0; i < 3; ++i) {
    cfg.extrinsic_attitude(i) = angles::from_degrees(cfg.extrinsic_attitude_deg(i));
  }
  field(cfg.extrinsic_position, "extrinsic_position", "m[3]");
  field(cfg.barometer_bias, "barometer_bias", "double");
}

void declare_config(SmootherConfig & cfg)
{
  using namespace config;
  name("SmootherConfig");
  field(cfg.lag, "lag", "s");
  field(cfg.relinearize_skip, "relinearize_skip");
  field(cfg.initial_sigma, "initial_sigma");
  field(cfg.relin_thresh, "relinearize_threshold");
  field(cfg.enable_relinearization, "enable_relinearization");
  field(cfg.evaluate_nonlinear_error, "evaluate_nonlinear_error");
  field(cfg.factorization_method, "factorization_method");
  field(cfg.cache_linearized_factors, "cache_linearized_factors");
  field(cfg.enable_detailed_results, "enable_detailed_results");
  field(cfg.enable_partial_relinearization_check, "enable_partial_relinearization_check");
  field(cfg.find_unused_factor_slots, "find_unused_factor_slots");
}

void declare_config(ImuPreintegrationConfig & cfg)
{
  using namespace config;
  name("ImuPreintegrationConfig");
  field(cfg.accel_noise_density, "accel_noise_density");
  field(cfg.accel_bias_random_walk, "accel_bias_random_walk");
  field(cfg.gyro_noise_density, "gyro_noise_density");
  field(cfg.gyro_bias_random_walk, "gyro_bias_random_walk");
  field(cfg.integration_covariance, "integration_covariance");
  field(cfg.bias_acc_omega_int, "bias_acc_omega_int");
}

void declare_config(GraphConfig & cfg)
{
  using namespace config;
  name("GraphConfig");
  field(cfg.log_directory, "log_directory");
  {
    NameSpace ns{"graph"};
    field(cfg.log_level, "log_level", "trace|debug|info|warn|error|critical");
    field(cfg.additional_iterations, "additional_iterations");
    field(cfg.world_frame, "world_frame");
    field(cfg.body_frame, "body_frame");
    field(cfg.use_fixed_lag, "use_fixed_lag");
    field(cfg.smoother, "smoother");
  }
  {
    NameSpace ns{"imu"};
    field(cfg.accel_bias_prior, "accel_bias_prior");
    field(cfg.gyro_bias_prior, "gyro_bias_prior");
    field(cfg.preint, "preintegration");
  }
  field(cfg.barometer, "barometer");

  // TODO: verify params
}

GraphManager::GraphManager(ros::NodeHandle & pnh, std::shared_ptr<ImuManager> imu_mgr_ptr)
: config_(config::checkValid(config::fromRos<GraphConfig>(pnh))), imu_mgr_ptr_(imu_mgr_ptr)
{
  // Create logging sinks
  logger_console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_console_sink_->set_level(spdlog::level::trace);
  logger_file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    config_.log_directory + "graph_manager.log", true);
  logger_file_sink_->set_level(spdlog::level::trace);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_unique<spdlog::logger>("graph_manager", sinks.begin(), sinks.end());
  logger_->set_level(static_cast<spdlog::level::level_enum>(config_.log_level));
  logger_->flush_on(static_cast<spdlog::level::level_enum>(config_.log_level));

  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> odom_logger_file_sink =
    std::make_shared<spdlog::sinks::basic_file_sink_mt>("rig_odometry.log", true);
  odom_logger_file_sink->set_level(spdlog::level::trace);
  odom_logger_ = std::make_unique<spdlog::logger>("odometry", odom_logger_file_sink);
  odom_logger_->set_pattern("%v");
  odom_logger_->info("timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz");

  logger_->info("Initialized with params:\n {}", config::toString(config_));

  logger_->debug("Setting up graph parameters");
  // Setup graph manager parameters
  is_graph_initialized_ = false;
  current_graph_key_ = 0;

  // Setup iSAM2 parameters
  isam2_params_.setOptimizationParams(gtsam::ISAM2GaussNewtonParams());
  gtsam::FastMap<char, gtsam::Vector> relinearize_threshold;
#if USE_POS
  relinearize_threshold['x'] =
    (gtsam::Vector(6) << config_.smoother.relin_thresh.attitude,
     config_.smoother.relin_thresh.attitude, config_.smoother.relin_thresh.attitude,
     config_.smoother.relin_thresh.position, config_.smoother.relin_thresh.position,
     config_.smoother.relin_thresh.position)
      .finished();
#else
  relinearize_threshold['x'] =
    (gtsam::Vector(3) << config_.smoother.relin_thresh.attitude,
     config_.smoother.relin_thresh.attitude, config_.smoother.relin_thresh.attitude)
      .finished();
#endif
  relinearize_threshold['v'] =
    (gtsam::Vector(3) << config_.smoother.relin_thresh.velocity,
     config_.smoother.relin_thresh.velocity, config_.smoother.relin_thresh.velocity)
      .finished();
  relinearize_threshold['b'] =
    (gtsam::Vector(6) << config_.smoother.relin_thresh.accel_bias,
     config_.smoother.relin_thresh.accel_bias, config_.smoother.relin_thresh.accel_bias,
     config_.smoother.relin_thresh.gyro_bias, config_.smoother.relin_thresh.gyro_bias,
     config_.smoother.relin_thresh.gyro_bias)
      .finished();
  relinearize_threshold['e'] =
    (gtsam::Vector(6) << config_.smoother.relin_thresh.extrinsic_attitude,
     config_.smoother.relin_thresh.extrinsic_attitude,
     config_.smoother.relin_thresh.extrinsic_attitude,
     config_.smoother.relin_thresh.extrinsic_position,
     config_.smoother.relin_thresh.extrinsic_position,
     config_.smoother.relin_thresh.extrinsic_position)
      .finished();
  relinearize_threshold['a'] = (gtsam::Vector(1) << 0.0).finished();  // TODO: Add

  isam2_params_.setRelinearizeThreshold(relinearize_threshold);
  isam2_params_.relinearizeSkip = config_.smoother.relinearize_skip;
  isam2_params_.enableRelinearization = config_.smoother.enable_relinearization;
  isam2_params_.evaluateNonlinearError = config_.smoother.evaluate_nonlinear_error;
  isam2_params_.setFactorization(config_.smoother.factorization_method);
  isam2_params_.cacheLinearizedFactors = config_.smoother.cache_linearized_factors;
  isam2_params_.enableDetailedResults = config_.smoother.enable_detailed_results;
  isam2_params_.enablePartialRelinearizationCheck =
    config_.smoother.enable_partial_relinearization_check;
  isam2_params_.findUnusedFactorSlots = config_.smoother.find_unused_factor_slots;

  if (!config_.use_fixed_lag) {
    // Create iSAM2 Object using the set parameters
    isam2_ = gtsam::ISAM2(isam2_params_);
    logger_->info("Printing iSAM2 Params: ");
  } else {
    ifl_ = gtsam::IncrementalFixedLagSmoother(config_.smoother.lag, isam2_params_);
    logger_->info("Printing iFL (iSAM2) Params: ");
  }
  isam2_params_.print();

  // Setup odom message and transform to be sent
  tf_stamped_.header.frame_id = config_.world_frame;
  tf_stamped_.child_frame_id = config_.body_frame;
  pose_stamped_.header.frame_id = config_.world_frame;
  path_.header.frame_id = config_.world_frame;

  pub_odom_ = pnh.advertise<nav_msgs::Odometry>("graph/odometry", 10, true);
  pub_odom_path_ = pnh.advertise<nav_msgs::Path>("graph/odom_path", 10, true);
  pub_pose_stamped_ = pnh.advertise<geometry_msgs::PoseStamped>("graph/pose_stamped", 10, true);
  pub_baro_bias_ = pnh.advertise<sensor_msgs::FluidPressure>("graph/bias/baro", 10, false);
  pub_graph_debug_ = pnh.advertise<rig_msgs::GraphDebug>("graph/debug", 10, false);
  pub_extrinsic_ = pnh.advertise<geometry_msgs::PoseStamped>("graph/extrinsic", 10, false);
}

GraphManager::~GraphManager() {}

bool GraphManager::isGraphInitialized() { return is_graph_initialized_; }

void GraphManager::initializeGraph(
  const double ts, const gtsam::Key start_key, const gtsam::Pose3 & init_pose,
  const gtsam::Pose3 & init_extrinsic, const double & init_altitude)
{
  if (ts <= 0.0) {
    logger_->error("InitializeGraph: Invalid timestamp: {}", ts);
    return;
  }

  logger_->info("Initializing graph");
  // Initialize values
  gtsam::Values initial_estimate;
#if USE_POS
  initial_estimate.insert(X(start_key), init_pose);
#else
  initial_estimate.insert(X(start_key), init_pose.rotation());
#endif
  initial_estimate.insert(V(start_key), gtsam::Vector3(0, 0, 0));
  initial_estimate.insert(B(start_key), *imu_bias_prior_);
  initial_estimate.insert(E(start_key), init_extrinsic);
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config_.smoother.initial_sigma.attitude,
     config_.smoother.initial_sigma.position)
      .finished());
  gtsam::noiseModel::Diagonal::shared_ptr rot_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << config_.smoother.initial_sigma.attitude).finished());
  gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model =
    gtsam::noiseModel::Diagonal::Sigmas(config_.smoother.initial_sigma.velocity);  // m/s
  gtsam::noiseModel::Diagonal::shared_ptr imu_bias_noise_model =
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << config_.smoother.initial_sigma.accel_bias,
       config_.smoother.initial_sigma.gyro_bias)
        .finished());  // Unsure of order of acc and gyr
  auto extrinsic_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config_.smoother.initial_sigma.extrinsic_attitude,
     config_.smoother.initial_sigma.extrinsic_position)
      .finished());
// Add prior factor to update graph
#if USE_POS
  new_factors_.addPrior(X(start_key), init_pose, pose_noise_model);
#else
  new_factors_.addPrior(X(start_key), init_pose.rotation(), rot_noise_model);
#endif
  new_factors_.addPrior(V(start_key), gtsam::Vector3(0, 0, 0), velocity_noise_model);
  new_factors_.addPrior(B(start_key), *imu_bias_prior_, imu_bias_noise_model);
  new_factors_.addPrior(E(start_key), init_extrinsic, extrinsic_noise_model);

  if (config_.barometer.enabled) {
    initial_estimate.insert(A(start_key), init_altitude);
    gtsam::noiseModel::Diagonal::shared_ptr altitude_bias_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(1) << config_.smoother.initial_sigma.barometer_bias).finished());
    new_factors_.addPrior(A(start_key), init_altitude, altitude_bias_noise_model);
  }

  // has to be set before first optimize (called in AddValuesToKeyTsMap and UpdateCurrentState)
  first_graph_key_ = start_key;
  // iFL: Fill up new_key_ts_map_
  if (config_.use_fixed_lag) addValuesToKeyTsMap(initial_estimate, ts);
  // Add update graph and inititial estimates to isam2's internal graph
  runUpdateStep(initial_estimate);
  // Reset the new factors
  clearNewFactors();
  if (config_.use_fixed_lag) clearNewKeyTsMap();
  // Update current state
  updateCurrentState(start_key, ts);
  is_graph_initialized_ = true;
  logger_->info("Graph Initialized");
}

void GraphManager::initializeImuPreintegrator(
  double gravity_magnitude, const Eigen::Matrix4d & T_B_I,
  const gtsam::Vector3 & gyro_bias_estimate)
{
  imu_preintegrator_params_ = PreintegratedMeasurements::Params::MakeSharedU(gravity_magnitude);
  gtsam::Pose3 body_P_imu;
  Convert(T_B_I, body_P_imu);
  imu_preintegrator_params_->setBodyPSensor(body_P_imu);
  imu_preintegrator_params_->setAccelerometerCovariance(
    gtsam::Matrix33::Identity() * std::pow(config_.preint.accel_noise_density, 2));
  imu_preintegrator_params_->setBiasAccCovariance(
    gtsam::Matrix33::Identity() * std::pow(config_.preint.accel_bias_random_walk, 2));
  imu_preintegrator_params_->setGyroscopeCovariance(
    gtsam::Matrix33::Identity() * std::pow(config_.preint.gyro_noise_density, 2));
  imu_preintegrator_params_->setBiasOmegaCovariance(
    gtsam::Matrix33::Identity() * std::pow(config_.preint.gyro_bias_random_walk, 2));
  imu_preintegrator_params_->setIntegrationCovariance(
    gtsam::Matrix33::Identity() * config_.preint.integration_covariance);
  imu_preintegrator_params_->setBiasAccOmegaInit(
    gtsam::Matrix66::Identity() * config_.preint.bias_acc_omega_int);

  // TODO: fix addition logic
  imu_bias_prior_ = std::make_shared<gtsam::imuBias::ConstantBias>(
    config_.accel_bias_prior, config_.gyro_bias_prior + gyro_bias_estimate);
  imu_preintegrator_ =
    std::make_shared<PreintegratedMeasurements>(imu_preintegrator_params_, *imu_bias_prior_);
  logger_->info("Initialized IMU preintegrator with:");
  imu_preintegrator_params_->print("Parameters:\t");
  imu_bias_prior_->print("IMU Bias Prior:\t");
}

void GraphManager::updateImuPreintegrator(const ImuMap & imu_measurements)
{
  if (imu_measurements.size() < 2) {
    // This case should be handled by the caller function. This imu measurement has already been integrated
    logger_->error("Preintegration not possible as there are less than 2 measurements");
    return;
  }

  // Remove the previous integration results from the preintegrator
  imu_preintegrator_->resetIntegrationAndSetBias(current_state_.imu_bias());

  // Pairwise integration of the measurements. As the imu_measurements are never skipped, the first
  // measurement has always already been integrated in the previous step
  auto prev_itr = imu_measurements.begin();
  auto curr_itr = prev_itr;
  ++curr_itr;

  for (; curr_itr != imu_measurements.end(); ++curr_itr, ++prev_itr) {
    double dt = curr_itr->first - prev_itr->first;
    imu_preintegrator_->integrateMeasurement(
      curr_itr->second.head<3>(), curr_itr->second.tail<3>(), dt);
  }
}

void GraphManager::getImuPropagatedEstimate(
  gtsam::Values & estimate, const gtsam::Key key, gtsam::Pose3 & pose)
{
  gtsam::NavState imu_propagated_state =
    imu_preintegrator_->predict(current_state_.nav_state(), current_state_.imu_bias());
#if USE_POS
  estimate.insert(X(key), imu_propagated_state.pose());
#else
  estimate.insert(X(key), imu_propagated_state.pose().rotation());
#endif
  estimate.insert(V(key), imu_propagated_state.velocity());
  estimate.insert(B(key), current_state_.imu_bias());

  pose = imu_propagated_state.pose();
}

gtsam::Matrix GraphManager::getCovariance(const gtsam::Key key)
{
  if (!config_.use_fixed_lag) {
    return isam2_.marginalCovariance(key);
  } else {
    return ifl_.marginalCovariance(key);
  }
}

void GraphManager::publishResults(
  const double ts, const gtsam::Matrix & pose_covar, const gtsam::Matrix & vel_covar)
{
  const ros::Time stamp(ts);
  // extract necessary information from current state
  const gtsam::Pose3 pose = current_state_.nav_state().pose();
  const gtsam::Point3 position = pose.translation();
  const gtsam::Rot3 R_body_world = pose.rotation();
  const gtsam::Quaternion quaternion = R_body_world.toQuaternion();
  const gtsam::Vector3 velocity = current_state_.nav_state().velocity();
  const gtsam::imuBias::ConstantBias imu_bias = current_state_.imu_bias();
  const gtsam::Pose3 extrinsic = current_state_.extrinsic();
  const double baro_bias = current_state_.baro_bias();

  // Fill up odom_ with new values
  // Note: the function is using the current_state_ directly. This may not be safe when moving to a
  // multithreaded approach
  tf2::Transform t_W_G;
  Convert(pose, t_W_G);
  PublishOdometry(
    pub_odom_, config_.world_frame, config_.body_frame, stamp, t_W_G,
    R_body_world.unrotate(velocity), pose_covar,
    R_body_world.transpose() * vel_covar * R_body_world.matrix());  // TODO fix

  // Publish TF
  tf_stamped_.header.stamp = stamp;
  Convert(t_W_G, tf_stamped_);
  tf2_broadcaster_.sendTransform(tf_stamped_);

  // Add pose to path
  PublishPath(pub_odom_path_, stamp, t_W_G, path_);

  if (path_.poses.size()) {
    pose_stamped_ = path_.poses.back();
    pub_pose_stamped_.publish(pose_stamped_);
  }

  // Publish the estimated
  sensor_msgs::FluidPressure baro_bias_msg;
  baro_bias_msg.header.stamp = stamp;
  // misusing message definitions
  baro_bias_msg.fluid_pressure = barometry::pressure_from_height(baro_bias);
  baro_bias_msg.variance = baro_bias;
  pub_baro_bias_.publish(baro_bias_msg);

  // Publish debug topic
  {
    rig_msgs::GraphDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.body_frame;
    Convert(imu_bias.accelerometer(), msg.accel_bias);
    Convert(imu_bias.gyroscope(), msg.gyro_bias);
    pub_graph_debug_.publish(msg);
  }

  {
    // Publish extrinsic topic
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.body_frame;
    Convert(extrinsic, msg.pose);
    pub_extrinsic_.publish(msg);
  }

  {
    // publish extrinsic TF
    geometry_msgs::TransformStamped ext_tfs;
    ext_tfs.header.stamp = stamp;
    ext_tfs.header.frame_id = config_.body_frame;
    ext_tfs.child_frame_id = "radar";
    const gtsam::Point3 ext_p = extrinsic.translation();
    const gtsam::Quaternion ext_q = extrinsic.rotation().toQuaternion();
    ext_tfs.transform.translation.x = ext_p.x();
    ext_tfs.transform.translation.y = ext_p.y();
    ext_tfs.transform.translation.z = ext_p.z();
    ext_tfs.transform.rotation.w = ext_q.w();
    ext_tfs.transform.rotation.x = ext_q.x();
    ext_tfs.transform.rotation.y = ext_q.y();
    ext_tfs.transform.rotation.z = ext_q.z();
    tf2_broadcaster_.sendTransform(ext_tfs);
  }

  // log to file
  odom_logger_->info(
    "{},{},{},{},{},{},{},{},{},{},{}", ts, position.x(), position.y(), position.z(),
    quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(), velocity.x(), velocity.y(),
    velocity.z());
}

#if USE_POS
gtsam::CombinedImuFactor
#else
gtsam::CombinedImuFactor2
#endif
GraphManager::addImuFactor(
  const gtsam::Key old_key, const gtsam::Key new_key, const ImuMap & imu_measurements)
{
  updateImuPreintegrator(imu_measurements);
  ImuFactor combined_imu_factor(
    X(old_key), V(old_key), X(new_key), V(new_key), B(old_key), B(new_key), *imu_preintegrator_);
  new_factors_.add(combined_imu_factor);
  return combined_imu_factor;
}

void GraphManager::addBarometerFactors(
  const gtsam::Key & prev_key, const gtsam::Key & key, const double & barometric_pressure,
  const double & sigma_height, const double & sigma_bias,
  const gtsam::noiseModel::mEstimator::Base::shared_ptr & robust)
{
  double altitude = barometry::height_from_pressure(barometric_pressure);

  const auto noise_meas = gtsam::noiseModel::Isotropic::Sigma(1, sigma_height);
  const gtsam::SharedNoiseModel robust_noise_meas =
    gtsam::noiseModel::Robust::Create(robust, noise_meas);
  rig::DifferentialBarometryFactor factor(altitude, X(key), A(key), robust_noise_meas);

  const auto noise_rw = gtsam::noiseModel::Isotropic::Sigma(1, sigma_bias);
  gtsam::BetweenFactor<double> bias_random_walk(A(prev_key), A(key), 0.0, noise_rw);

  new_factors_.add(factor);
  new_factors_.add(bias_random_walk);
}

void GraphManager::addDopplerFactor(
  const gtsam::Point3 & point, const double & doppler, const gtsam::Pose3 & pose_R_B,
  const gtsam::Vector3 & angular_velocity_B, const gtsam::Key & key, const double & sigma,
  const double & huber_threshold)
{
  const gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(huber_threshold),
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(sigma)));

  rig::DopplerFactor factor(
    point, doppler, pose_R_B, angular_velocity_B, X(key), V(key), B(key), noise_model);

  new_factors_.add(factor);
}

void GraphManager::addFactors(const gtsam::NonlinearFactorGraph & new_factors)
{
  new_factors_.add(new_factors);
}

void GraphManager::runUpdateStep(const gtsam::Values & initial_estimate)
{
  logger_->trace("running update step");
  try {
    if (!config_.use_fixed_lag) {
      // iSAM2
      isam2_.update(new_factors_, initial_estimate);
      for (uint_least8_t i = 0; i < config_.additional_iterations; ++i) isam2_.update();
      current_values_ = isam2_.calculateEstimate();
    } else {
      // iFL
      ifl_.update(new_factors_, initial_estimate, new_key_ts_map_);
      for (uint_least8_t i = 0; i < config_.additional_iterations; ++i) ifl_.update();
      current_values_ = ifl_.calculateEstimate();
    }
  } catch (const std::exception & e) {
    logger_->error("Exception in RunUpdateStep: {}", e.what());
    // PrintGraph(true);
    // PrintFullGraph(true);
    if (config_.use_fixed_lag) printIFLKeyTsMap();
  }
}

gtsam::Key GraphManager::getNewStateKey() { return ++current_graph_key_; }

void GraphManager::clearNewFactors()
{
  logger_->trace("clearing new factors");
  new_factors_.resize(0);
}

void GraphManager::clearNewKeyTsMap()
{
  logger_->trace("clearing new key ts map");
  if (!config_.use_fixed_lag) {
    logger_->warn("Will not clear new_key_ts_map as fixed lag is not used");
    return;
  }
  new_key_ts_map_.clear();
}

void GraphManager::addValuesToKeyTsMap(const gtsam::Values & values, const double ts)
{
  if (!config_.use_fixed_lag) {
    logger_->warn("Will not add values to new_key_ts_map as fixed lag is not used");
    return;
  }
  for (const gtsam::Values::ConstKeyValuePair & value : values) new_key_ts_map_[value.key] = ts;
  // add extrinsic key every time to ensure it doesn't get marginalized
  new_key_ts_map_[E(first_graph_key_)] = ts;
}

void GraphManager::printIFLKeyTsMap()
{
  if (!config_.use_fixed_lag) {
    logger_->warn("Printing key_ts_map in the IFL. As fixed lag is not used this should be empty.");
  }
  logger_->info("Keys in the iFL: ");
  for (const gtsam::FixedLagSmoother::KeyTimestampMap::value_type & key_ts : ifl_.timestamps()) {
    logger_->info("Key: {} ts: {}", gtsam::DefaultKeyFormatter(key_ts.first), key_ts.second);
  }
}

void GraphManager::updateCurrentState(
  const gtsam::Key key, const double ts, const gtsam::Point3 & position)
{
  logger_->trace("updating current state");
  // key and ts that actually gets used
  gtsam::Key key_current;
  double ts_current;
  if (ts < current_state_.ts()) {
    logger_->warn(
      "Update called with a timestamp earlier than the current state. Updating with the optimized "
      "latest state. ts: {} current_state_ts: {}",
      ts, current_state_.ts());
    // The update is being called by a state that is before the last state in the graph
    // Update the current state at the same key and ts with the new optimzed value
    key_current = current_state_.key();
    ts_current = current_state_.ts();
  } else {
    // Update is correctly called as the last state in the graph
    key_current = key;
    ts_current = ts;
  }

#if USE_POS
  gtsam::Pose3 pose = current_values_.at<gtsam::Pose3>(X(key_current));
#else
  gtsam::Pose3 pose(current_values_.at<gtsam::Rot3>(X(key_current)), position);
#endif
  gtsam::Vector3 velocity = current_values_.at<gtsam::Vector3>(V(key_current));
  gtsam::imuBias::ConstantBias imu_bias =
    current_values_.at<gtsam::imuBias::ConstantBias>(B(key_current));
  gtsam::Pose3 extrinsic = current_values_.at<gtsam::Pose3>(E(first_graph_key_));

  logger_->trace("Updating using keys: [{}, {}]", key_current, first_graph_key_);
  current_state_.updateNavStateAndImuBias(
    key_current, ts_current, gtsam::NavState(pose, velocity), imu_bias);
  current_state_.updateExtrinsic(key_current, ts_current, extrinsic);

  if (config_.barometer.enabled) {
    const double baro_bias = current_values_.at<double>(A(key_current));
    current_state_.updateBaroBias(key_current, ts_current, baro_bias);
  }
}

void GraphManager::resetStateKeyIndex() { current_graph_key_ = 0; }

double GraphManager::getCurrentTimestamp() const { return current_state_.ts(); }

const State & GraphManager::getCurrentState() const { return current_state_; }

gtsam::Key GraphManager::getCurrentKey() const { return current_state_.key(); }

gtsam::Key GraphManager::getFirstKey() const { return first_graph_key_; }

void GraphManager::getCurrentValues(gtsam::Values & values) { values = current_values_; }

}  // namespace rig
