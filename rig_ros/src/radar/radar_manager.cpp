#include "rig/radar/radar_manager.hpp"

namespace rig
{
void declare_config(RangeFilterConfig & cfg)
{
  using namespace config;
  name("RangeFilterConfig");
  field(cfg.min, "min", "m");
  field(cfg.max, "max", "m");
}

void declare_config(RadarFilterConfig & cfg)
{
  using namespace config;
  name("RadarFilterConfig");
  field(cfg.range, "range");
  field(cfg.min_db, "min_db");
  field(cfg.azimuth_threshold_deg, "azimuth_threshold_deg");
  cfg.azimuth_threshold = angles::from_degrees(cfg.azimuth_threshold_deg);
  field(cfg.elevation_threshold_deg, "elevation_threshold_deg");
  cfg.elevation_threshold = angles::from_degrees(cfg.elevation_threshold_deg);
}

void declare_config(GravityConfig & cfg)
{
  using namespace config;
  name("GravityConfig");
  field(cfg.aligned_initialization, "aligned_initialization");
  field(cfg.estimate, "estimate");
  field(cfg.magnitude, "magnitude");
}

void declare_config(RadarSensorConfig & cfg)
{
  using namespace config;
  name("RadarSensorConfig");
  field(cfg.l_BR_B, "l_BR_B", "x, y, z");
  field(cfg.q_R_B, "q_R_B", "x, y, z, w");
  field(cfg.frame_time_ms, "frame_time_ms", "ms");
  cfg.frame_time = cfg.frame_time_ms * 1.0e-3;
  field(cfg.range_sigma, "range_sigma", "m");
  field(cfg.doppler_resolution, "doppler_resolution", "m/s");
  cfg.doppler_sigma =
    cfg.doppler_resolution / std::sqrt(12);  // assuming uniform to normal distribution conversion
  field(cfg.doppler_max, "doppler_max", "m/s");
  field(cfg.phase_resolution_deg, "phase_resolution_deg", "deg");
  cfg.phase_sigma = angles::from_degrees(cfg.phase_resolution_deg) / std::sqrt(12);
}

void declare_config(DopplerRobustConfig & cfg)
{
  using namespace config;
  name("DopplerRobustConfig");
  field(cfg.lesser, "lesser");
  field(cfg.greater, "greater");
}

void declare_config(DopplerConfig & cfg)
{
  using namespace config;
  name("DopplerConfig");
  field(cfg.robust_cost, "robust_cost", "none|fair|huber|etc");
}

void declare_config(PointConfig & cfg)
{
  using namespace config;
  name("PointConfig");
  field(cfg.enabled, "enabled");
  field(cfg.robust_cost, "robust_cost", "none|fair|huber|etc");
}

void declare_config(MapConfig & cfg)
{
  using namespace config;
  name("MapConfig");
  field(cfg.enabled, "enabled");
  field(cfg.n_scans_delay, "n_scans_delay");
  field(cfg.voxel_grid_leaf_size, "voxel_grid_leaf_size");
  field(cfg.search_radius, "search_radius");
  field(cfg.minimum_neighbors, "minimum_neighbors");
}

void declare_config(RadarConfig & cfg)
{
  using namespace config;
  name("RadarConfig");
  field(cfg.log_directory, "log_directory");
  {
    NameSpace ns{"radar"};
    field(cfg.log_level, "log_level", "trace|debug|info|warn|error|critical");
    field(cfg.use_angular_velocity, "use_angular_velocity");
    field(cfg.exp_avg_angvel, "exp_avg_angvel");
    field(cfg.angle_noise, "use_angle_noise");
    field(cfg.filter, "filter");
    field(cfg.estimate_extrinsic, "estimate_extrinsic");
    field(cfg.doppler, "doppler");
    field(cfg.point, "point");
    field(cfg.map, "map");
  }
  field(cfg.sensor, "sensor");
  {
    NameSpace ns{"graph"};
    field(cfg.use_fixed_lag, "use_fixed_lag");
  }
  field(cfg.gravity, "gravity");
  field(cfg.barometer, "barometer");

  // TODO verify params
}

RadarManager::RadarManager(ros::NodeHandle & pnh) : preint_init_(false)
{
  // TODO find better place to set this parameter
  config::Settings().print_missing = true;
  config_ = config::checkValid(config::fromRos<RadarConfig>(pnh));

  initializeLogger();
  logger_->info("Initialized with params:\n {}", config::toString(config_));

  imu_mgr_ptr_ = std::make_shared<ImuManager>(pnh);
  graph_mgr_ptr_ = std::make_unique<GraphManager>(pnh, imu_mgr_ptr_);
  if (config_.map.enabled) {
    mapper_ptr_ = std::make_shared<Mapper>(config_.map);
  }
  if (config_.point.enabled) {
    if (!config_.map.enabled) {
      throw std::invalid_argument("Point enabled but not map");
    }
  }

  assignRobustCost(
    config_.doppler.robust_cost.lesser, doppler_lesser_robust_cost_ptr_, "doppler less: ");
  assignRobustCost(
    config_.doppler.robust_cost.greater, doppler_greater_robust_cost_ptr_, "doppler greater: ");
  if (config_.point.enabled) {
    assignRobustCost(config_.point.robust_cost, point_robust_cost_ptr_, "point: ");
  }
  if (config_.barometer.enabled) {
    assignRobustCost(config_.barometer.robust_cost, barometer_robust_cost_ptr_, "barometer: ");
  }

  // Publishers
  pub_radar_debug_ = pnh.advertise<rig_msgs::RadarDebug>("debug", 10);
  pub_cloud_filter_ = pnh.advertise<sensor_msgs::PointCloud2>("cloud/filter", 10);
  pub_cloud_map_ = pnh.advertise<sensor_msgs::PointCloud2>("cloud/map", 10);
  pub_cloud_gaussians_ = pnh.advertise<visualization_msgs::MarkerArray>("cloud/gaussians", 10);
  pub_cloud_lesser_ = pnh.advertise<sensor_msgs::PointCloud2>("cloud/lesser", 10);
  pub_cloud_greater_ = pnh.advertise<sensor_msgs::PointCloud2>("cloud/greater", 10);

  pub_odom_ = pnh.advertise<nav_msgs::Odometry>("odometry", 100, false);
  pub_pose_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 100, false);
  pub_twist_ = pnh.advertise<geometry_msgs::TwistStamped>("twist", 100, false);
  pub_debug_twist_ = pnh.advertise<geometry_msgs::TwistStamped>("debug/twist", 100, false);
  preint_params_ =
    gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config_.gravity.magnitude);
  preint_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
    preint_params_, gtsam::imuBias::ConstantBias());

  // Subscribers
  logger_->debug("Creating Subscriptions");
  sub_baro_ =
    pnh.subscribe<sensor_msgs::FluidPressure>("/baro", 100, &RadarManager::baroCallback, this);
  sub_imu_ = pnh.subscribe<sensor_msgs::Imu>("/imu", 100, &RadarManager::imuCallback, this);
  sub_cloud_ =
    pnh.subscribe<sensor_msgs::PointCloud2>("/cloud", 100, &RadarManager::cloudCallback, this);
}

RadarManager::~RadarManager() { logger_->debug("Destructor called"); }

// TODO: make function for all managers
void RadarManager::initializeLogger()
{  // Create logging sinks
  // Sinks are set to trace so that they will always capture anything written to them. The control is on the logger level
  logger_console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_console_sink_->set_level(spdlog::level::trace);
  logger_file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    config_.log_directory + "radar_manager.log", true);
  logger_file_sink_->set_level(spdlog::level::trace);

  // Create logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(logger_console_sink_);
  sinks.push_back(logger_file_sink_);
  logger_ = std::make_unique<spdlog::logger>("radar_manager", sinks.begin(), sinks.end());
  logger_->set_level(static_cast<spdlog::level::level_enum>(config_.log_level));
  logger_->flush_on(static_cast<spdlog::level::level_enum>(config_.log_level));
}

void RadarManager::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  // correct timestamp
  const double ts = msg->header.stamp.toSec();

  // add to imu buffer
  imu_mgr_ptr_->addToImuBuffer(
    ts, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  // integrate measurement and publish result
  std::lock_guard<std::mutex> lock(preint_mutex_);
  if (preint_init_) {
    const double dt = ts - prev_ts_;

    logger_->debug(
      "Integrating (dt: {}) with bias\n\tbg: {}\n\tbg: {}", dt * 1e3,
      preint_->biasHat().gyroscope().transpose(), preint_->biasHat().accelerometer().transpose());
    if (dt > 7.5e-3) {
      logger_->warn("Integrating (dt: {})", dt * 1e3);
    }

    const Eigen::Vector3d linacc(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d angvel(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    preint_->integrateMeasurement(linacc, angvel, dt);
    const gtsam::NavState new_state = preint_->predict(hr_state_, preint_->biasHat());

    const Eigen::Vector3d angvel_corrected = preint_->biasHat().correctGyroscope(angvel);
    const Eigen::Vector3d linvel_body = new_state.bodyVelocity();

    // publish odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.child_frame_id = msg->header.frame_id;
    odom_msg.pose.pose.position.x = new_state.position().x();
    odom_msg.pose.pose.position.y = new_state.position().y();
    odom_msg.pose.pose.position.z = new_state.position().z();
    odom_msg.pose.pose.orientation.w = new_state.quaternion().w();
    odom_msg.pose.pose.orientation.x = new_state.quaternion().x();
    odom_msg.pose.pose.orientation.y = new_state.quaternion().y();
    odom_msg.pose.pose.orientation.z = new_state.quaternion().z();
    odom_msg.twist.twist.linear.x = linvel_body.x();
    odom_msg.twist.twist.linear.y = linvel_body.y();
    odom_msg.twist.twist.linear.z = linvel_body.z();
    odom_msg.twist.twist.angular.x = angvel_corrected.x();
    odom_msg.twist.twist.angular.y = angvel_corrected.y();
    odom_msg.twist.twist.angular.z = angvel_corrected.z();
    pub_odom_.publish(odom_msg);

    // publish pose stamped
    geometry_msgs::PoseStamped ps_msg;
    ps_msg.header = odom_msg.header;
    ps_msg.pose = odom_msg.pose.pose;
    pub_pose_.publish(ps_msg);

    // TODO: publish world frame twist
    // publish twist stamped
    geometry_msgs::TwistStamped ts_msg;
    ts_msg.header = odom_msg.header;
    ts_msg.twist = odom_msg.twist.twist;
    pub_twist_.publish(ts_msg);
  }

  prev_ts_ = ts;
}

void RadarManager::cloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
  // correct time stamp
  const double ts = msg->header.stamp.toSec() + 0.5 * config_.sensor.frame_time;

  if (!graph_mgr_ptr_->isGraphInitialized()) {
    // try to initialize
    const gtsam::Key key = graph_mgr_ptr_->getNewStateKey();
    initialize(ts, key);
  } else {
    // check timestamp for validity
    static ros::Time prev_ts = ros::Time(0);
    if (msg->header.stamp == prev_ts) {
      logger_->error("cloud with same stamp as previous");
      return;
    }
    prev_ts = msg->header.stamp;

    // check if cloud empty
    if (msg->data.empty()) {
      logger_->error("No points in scan");
    }

    // check cloud type
    static bool is_first_cloud = true;
    if (is_first_cloud) {
      is_first_cloud = false;

      std::set<std::string> fields;
      for (const auto & field : msg->fields) {
        fields.emplace(field.name);
      }

      if (
        fields.find("x") != fields.end() && fields.find("y") != fields.end() &&
        fields.find("z") != fields.end() && fields.find("snr_db") != fields.end() &&
        fields.find("noise_db") != fields.end() && fields.find("v_doppler_mps") != fields.end()) {
        logger_->debug("Doer cloud type");

        // fixing frame convention
        const Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitZ());
        // const Eigen::Quaterniond ned2enu(0, -std::sqrt(2.0) / 2.0, -std::sqrt(2.0) / 2.0, 0);
        config_.sensor.q_R_B *= Eigen::Quaterniond(rot);
      } else {
        logger_->debug("mmWave cloud type");
      }
    }

    const gtsam::Key key = graph_mgr_ptr_->getNewStateKey();
    processCloud(*msg, ts, key);
  }
}

void RadarManager::baroCallback(const sensor_msgs::FluidPressureConstPtr & msg)
{
  imu_mgr_ptr_->addToBaroBuffer(msg->header.stamp.toSec(), msg->fluid_pressure);
}

void RadarManager::assignRobustCost(
  const std::string & param, gtsam::noiseModel::mEstimator::Base::shared_ptr & robust_cost_ptr,
  const std::string & text)
{
  // defaulting to 95% asymptotic effeciency paramter according to:
  // http://www-sop.inria.fr/odyssee/software/old_robotvis/Tutorial-Estim/node24.html
  if (param == "none") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Null::Create();
  } else if (param == "fair") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Fair::Create(1.3998);
  } else if (param == "huber") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Huber::Create(1.345);
  } else if (param == "cauchy") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Cauchy::Create(2.3849);
  } else if (param == "geman") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::GemanMcClure::Create(1.0);
  } else if (param == "welsch") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Welsch::Create(2.9846);
  } else if (param == "tukey") {
    robust_cost_ptr = gtsam::noiseModel::mEstimator::Tukey::Create(4.68519);
  } else {
    logger_->warn("Invalid robust cost function: {}", param);
    throw std::invalid_argument(param);
  }

  robust_cost_ptr->print(text);
}

void RadarManager::convertPointCloud(
  const sensor_msgs::PointCloud2 & msg, pcl::PointCloud<radar::mmWavePoint> & cloud)
{
  std::set<std::string> fields;

  for (const auto & field : msg.fields) {
    fields.emplace(field.name);
  }

  if (
    fields.find("x") != fields.end() && fields.find("y") != fields.end() &&
    fields.find("z") != fields.end() && fields.find("snr_db") != fields.end() &&
    fields.find("noise_db") != fields.end() && fields.find("v_doppler_mps") != fields.end()) {
    pcl::PointCloud<radar::rioPoint> temp;
    pcl::fromROSMsg(msg, temp);
    for (const auto & t : temp) {
      radar::mmWavePoint p;

      p.x = t.y;
      p.y = -t.x;
      p.z = t.z;
      p.velocity = t.v_doppler_mps;
      p.intensity = t.snr_db;

      cloud.points.push_back(p);
    }
    cloud.header = temp.header;
  } else {
    pcl::fromROSMsg(msg, cloud);
  }
}

void RadarManager::filterPointCloud(
  const sensor_msgs::PointCloud2 & cloud_msg, TargetVector & valid_targets)
{
  pcl::PointCloud<radar::mmWavePoint> cloud;
  convertPointCloud(cloud_msg, cloud);

  for (const auto & point : cloud.points) {
    // calculate metrics for filtering
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    const double azimuth = std::atan2(point.y, point.x);
    const double elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));

    if (
      (range >= config_.filter.range.min) && (range <= config_.filter.range.max) &&
      (std::abs(azimuth) <= config_.filter.azimuth_threshold) &&
      (std::abs(elevation) <= config_.filter.elevation_threshold) &&
      (point.intensity >= config_.filter.min_db)) {
      valid_targets.emplace_back(
        point.x, point.y, point.z, range, azimuth, elevation, point.velocity, point.intensity);
    }
  }

  logger_->debug(
    "Filtered point cloud size reduced from {} to {} ({} points removed)", cloud.points.size(),
    valid_targets.size(), cloud.points.size() - valid_targets.size());
}

bool RadarManager::initialize(const double & ts, const gtsam::Key & key)
{
  // Initialize graph
  // Get attitude for gravity aligned initialization
  gtsam::Rot3 init_attitude;
  double init_altitude = 0;
  double gravity_magnitude;
  gtsam::Vector3 gyro_bias_estimate(0.0, 0.0, 0.0);
  try {
    imu_mgr_ptr_->estimateAttitudeFromImu(ts, init_attitude, gravity_magnitude, gyro_bias_estimate);
  } catch (const std::length_error & e) {
    static double prev_time = 0;

    if (ts - prev_time > 1.0) {
      logger_->warn("{} : Skipping this measurement", e.what());
      prev_time = ts;
    }
    graph_mgr_ptr_->resetStateKeyIndex();
    return false;
  }

  if (!config_.gravity.estimate) {
    logger_->debug("Gravity magnitude set from param");
    gravity_magnitude = config_.gravity.magnitude;
  } else {
    logger_->debug("Gravity magnitude estimated");
  }
  logger_->info("Gravity magnitude is: {}", gravity_magnitude);

  if (!config_.gravity.aligned_initialization) {
    init_attitude = gtsam::Rot3::Identity();
  }

  if (config_.barometer.enabled) {
    imu_mgr_ptr_->estimateBarometerBias(ts, init_altitude);
  }

  // Initalize preintegration (this also sets up the imu to body tf) and graph
  graph_mgr_ptr_->initializeImuPreintegrator(
    gravity_magnitude, T_B_I_.matrix(), gyro_bias_estimate);

  gtsam::Pose3 init_pose(init_attitude, gtsam::Point3::Zero());
  gtsam::Pose3 init_extrinsic(gtsam::Rot3(config_.sensor.q_R_B), config_.sensor.l_BR_B);
  graph_mgr_ptr_->initializeGraph(ts, key, init_pose, init_extrinsic, init_altitude);

  logger_->info("Initialization complete");

  return true;
}

bool RadarManager::processCloud(
  const sensor_msgs::PointCloud2 & cloud_msg, const double & ts, const gtsam::Key & key)
{
  StopWatch sw_total;
  debug_msg_.header = cloud_msg.header;
  debug_msg_.n_points = cloud_msg.width;
  logger_->debug("Processing cloud from ts: {}", cloud_msg.header.stamp);
  const double prev_ts = graph_mgr_ptr_->getCurrentTimestamp();

  // TODO: make propogate function for this
  // add imu factor
  ImuMap imu_measurements;
  imu_mgr_ptr_->getInterpolatedImuMeasurements(prev_ts, ts, imu_measurements);
  graph_mgr_ptr_->addImuFactor(graph_mgr_ptr_->getCurrentKey(), key, imu_measurements);

  // get IMU prior
  gtsam::Values values;
  gtsam::Pose3 est_pose;
  graph_mgr_ptr_->getImuPropagatedEstimate(values, key, est_pose);

  // build baro factor
  if (config_.barometer.enabled) {
    double barometric_pressure;
    imu_mgr_ptr_->getInterpolatedBaroMeasurement(ts, barometric_pressure);
    graph_mgr_ptr_->addBarometerFactors(
      graph_mgr_ptr_->getCurrentKey(), key, barometric_pressure, config_.barometer.sigma,
      config_.barometer.bias_sigma, barometer_robust_cost_ptr_);
    values.insert<double>(A(key), graph_mgr_ptr_->getCurrentState().baro_bias());
  }

  // build radar factor
  const double ts_begin = ts - 0.5 * config_.sensor.frame_time;
  const double ts_end = ts + 0.5 * config_.sensor.frame_time;
  gtsam::Vector3 angvel_WB_B = gtsam::Vector3::Zero();
  if (config_.use_angular_velocity) {
    ImuMap imu_measurements;
    imu_mgr_ptr_->getInterpolatedImuMeasurements(ts_begin, ts_end, imu_measurements);

    // REVIEW: do coning corrections matter here?
    size_t number_of_measurements = 0;
    double duration = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    auto prev = imu_measurements.begin();
    auto curr = std::next(prev);
    for (; curr != imu_measurements.end(); ++prev, ++curr) {
      const double dt = curr->first - prev->first;
      duration += dt;
      const Eigen::Vector3d tau = curr->second.tail<3>() * dt;
      q = q * Eigen::Quaterniond(Eigen::AngleAxisd(tau.norm(), tau.normalized()));

      angvel_WB_B += curr->second.tail<3>();
      number_of_measurements++;
    }
    angvel_WB_B /= (double)number_of_measurements;
    if (config_.exp_avg_angvel) {
      const Eigen::AngleAxisd aa(q);
      angvel_WB_B = aa.axis() * aa.angle() / duration;
    }
  } else {
    angvel_WB_B = values.at<gtsam::imuBias::ConstantBias>(B(key)).gyroscope();
  }

  // filter point cloud
  StopWatch sw;
  TargetVector valid_targets;
  filterPointCloud(cloud_msg, valid_targets);
  debug_msg_.t_filter = sw.elapsed();
  debug_msg_.n_points_filter = valid_targets.size();
  if (valid_targets.size() < 3) {
    logger_->warn("Few points after filtering: {}", valid_targets.size());
  }

  // doppler factor
  sw.reset();
  boost::shared_ptr<DopplerExtrinsicHessianFactor> doppler_factor =
    boost::make_shared<DopplerExtrinsicHessianFactor>(
      valid_targets, angvel_WB_B, X(key), V(key), B(key), E(graph_mgr_ptr_->getFirstKey()),
      config_.sensor.doppler_sigma, config_.angle_noise ? config_.sensor.phase_sigma : 0.0,
      config_.sensor.doppler_max, doppler_lesser_robust_cost_ptr_, doppler_greater_robust_cost_ptr_,
      config_.estimate_extrinsic);
  gtsam::NonlinearFactorGraph new_factors;
  new_factors.add(doppler_factor);
  graph_mgr_ptr_->addFactors(new_factors);
  debug_msg_.t_graph_doppler = sw.elapsed();

  // distribution-distribution factor
  sw.reset();
  boost::shared_ptr<PointToDistributionHessianFactor> point_factor;
#if USE_POS
  if (config_.point.enabled && config_.map.enabled && mapper_ptr_->buildKdTree()) {
    // TODO: get static points from graph
    point_factor = boost::make_shared<PointToDistributionHessianFactor>(
      valid_targets, mapper_ptr_, X(key), E(graph_mgr_ptr_->getFirstKey()),
      config_.sensor.range_sigma, config_.sensor.phase_sigma, point_robust_cost_ptr_);

    gtsam::NonlinearFactorGraph new_factors;
    new_factors.add(point_factor);
    graph_mgr_ptr_->addFactors(new_factors);
  }
#endif
  debug_msg_.t_graph_gaussian = sw.elapsed();

  // publish
  publishDebugCloud(cloud_msg.header, valid_targets, pub_cloud_filter_);

  sw.reset();
  // TODO: make correct function for this
  if (config_.use_fixed_lag) graph_mgr_ptr_->addValuesToKeyTsMap(values, ts);
  // Update iSAM2 and current state
  graph_mgr_ptr_->runUpdateStep(values);
  graph_mgr_ptr_->clearNewFactors();
  if (config_.use_fixed_lag) graph_mgr_ptr_->clearNewKeyTsMap();
  graph_mgr_ptr_->updateCurrentState(key, ts, est_pose.translation());

  const State curr_state = graph_mgr_ptr_->getCurrentState();
  resetIntegrator(ts, curr_state.nav_state(), curr_state.imu_bias());
  debug_msg_.t_graph_update = sw.elapsed();

  graph_mgr_ptr_->publishResults(
    ts, graph_mgr_ptr_->getCovariance(X(key)), graph_mgr_ptr_->getCovariance(V(key)));

  if (doppler_factor != nullptr && config_.map.enabled) {
    mapper_ptr_->insert(
      doppler_factor->getStatic(),
      (curr_state.nav_state().pose() * curr_state.extrinsic()).matrix());
    debug_msg_.t_mapper_add = sw.elapsed();

    doppler_factor->updateAliasTargets();
    debug_msg_.n_points_lesser = doppler_factor->getLesser().size();
    debug_msg_.n_points_greater = doppler_factor->getGreater().size();

    publishDebugCloud(cloud_msg.header, doppler_factor->getLesser(), pub_cloud_lesser_);
    publishDebugCloud(cloud_msg.header, doppler_factor->getGreater(), pub_cloud_greater_);

    // publish radar linear velocity for debugging
    gtsam::Values current_values;
    graph_mgr_ptr_->getCurrentValues(current_values);
    const gtsam::Vector3 radar_velocity = doppler_factor->radarVelocity(current_values);

    geometry_msgs::TwistStamped tw_msg;
    tw_msg.header = cloud_msg.header;
    tw_msg.twist.linear.x = radar_velocity.x();
    tw_msg.twist.linear.y = radar_velocity.y();
    tw_msg.twist.linear.z = radar_velocity.z();
    pub_debug_twist_.publish(tw_msg);
  }

  if (config_.map.enabled) {
    std_msgs::Header header;
    header.stamp = cloud_msg.header.stamp;
    header.frame_id = "odom";
    publishCloud(header, *mapper_ptr_->getMap(), pub_cloud_map_);

    if (point_factor != nullptr) {
      publishGaussians(header, point_factor->getFeatures());
    }
  }

  debug_msg_.t_total = sw_total.elapsed();
  pub_radar_debug_.publish(debug_msg_);

  return true;
}

void RadarManager::publishDebugCloud(
  const std_msgs::Header & header, const TargetVector & targets, const ros::Publisher & pub)
{
  // convert to sensor_msgs point cloud
  pcl::PointCloud<radar::mmWavePoint> cloud;
  cloud.points.clear();
  for (const TargetData & t : targets) {
    radar::mmWavePoint p;
    p.x = t.x;
    p.y = t.y;
    p.z = t.z;
    p.velocity = t.radial_speed;
    p.intensity = t.intensity;

    cloud.points.emplace_back(p);
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;

  // publish
  pub.publish(msg);
}

template <typename PointT>
void RadarManager::publishCloud(
  const std_msgs::Header & header, const pcl::PointCloud<PointT> & cloud,
  const ros::Publisher & pub)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  pub.publish(msg);
}

void RadarManager::publishGaussians(
  const std_msgs::Header & header, const DistributionVector & gaussians)
{
  visualization_msgs::MarkerArray msg;
  msg.markers.reserve(gaussians.size());

  visualization_msgs::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.ns = "gaussians";
  marker.id = 0;
  marker.lifetime = ros::Duration(0.1);
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const Distribution & d : gaussians) {
    const Eigen::EigenSolver<Eigen::Matrix3d> es(d.covariance);

    marker.pose.position.x = d.mean.x();
    marker.pose.position.y = d.mean.y();
    marker.pose.position.z = d.mean.z();

    const Eigen::Vector3d sigmas = 3.0 * es.eigenvalues().real().cwiseSqrt();

    const Eigen::Vector3d eig_x = es.eigenvectors().real().col(0);
    const Eigen::Vector3d eig_y = es.eigenvectors().real().col(1);
    const Eigen::Vector3d eig_z = eig_x.cross(eig_y);
    Eigen::Matrix3d rotmat;
    rotmat.col(0) = eig_x;
    rotmat.col(1) = eig_y;
    rotmat.col(2) = eig_z;
    Eigen::Quaterniond quat(rotmat);  // should be transposed?

    marker.pose.orientation.w = quat.w();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.scale.x = sigmas.x();
    marker.scale.y = sigmas.y();
    marker.scale.z = sigmas.z();

    msg.markers.push_back(marker);

    marker.id++;
  }

  pub_cloud_gaussians_.publish(msg);
}

void RadarManager::resetIntegrator(
  const double ts, const gtsam::NavState & nav_state, const gtsam::imuBias::ConstantBias & bias)
{
  logger_->trace("resetting integrator");

  // TODO: improve naming, pretty unclear
  if (ts >= prev_ts_) {
    logger_->debug("resetIntegrator called with ts ({}) >= prev_ts_ ({}), skipping", ts, prev_ts_);
    return;
  }

  std::lock_guard<std::mutex> lock(preint_mutex_);
  hr_state_ = nav_state;
  preint_->resetIntegrationAndSetBias(bias);

  // find new imu measurements and integrate
  ImuMap imu_map;
  imu_mgr_ptr_->getInterpolatedImuMeasurements(ts, prev_ts_, imu_map);
  imu_map.erase(imu_map.begin());

  logger_->debug(
    "resetIntegrator with {} ms latency and {} imu measurements",
    (imu_map.rbegin()->first - ts) * 1e3, imu_map.size());

  double prev_ts = ts;
  for (const auto & kv : imu_map) {
    const double dt = kv.first - prev_ts;
    preint_->integrateMeasurement(kv.second.head<3>(), kv.second.tail<3>(), dt);

    prev_ts = kv.first;
  }

  preint_init_ = true;
}

}  // namespace rig
