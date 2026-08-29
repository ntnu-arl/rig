#pragma once

// C++
#include <mutex>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace rig
{
class State
{
private:
  gtsam::Key key_ = 0;
  double ts_ = 0.0;
  gtsam::NavState nav_state_;
  gtsam::imuBias::ConstantBias imu_bias_;
  double baro_bias_;
  gtsam::Pose3 extrinsic_;
  std::mutex mutex_;

public:
  State(){};

  State(const State & other)
  {
    key_ = other.key();
    ts_ = other.ts();
    nav_state_ = other.nav_state();
    imu_bias_ = other.imu_bias();
    baro_bias_ = other.baro_bias();
    extrinsic_ = other.extrinsic();
  };

  ~State(){};

  // Accessors
  const gtsam::Key & key() const { return key_; }

  const double & ts() const { return ts_; }

  const gtsam::NavState & nav_state() const { return nav_state_; }

  const gtsam::imuBias::ConstantBias & imu_bias() const { return imu_bias_; }

  const double & baro_bias() const { return baro_bias_; }

  const gtsam::Pose3 & extrinsic() const { return extrinsic_; }

  void updateKeyAndTimestamp(const gtsam::Key key, const double ts)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
  }

  void updateNavState(const gtsam::Key key, const double ts, const gtsam::NavState & nav_state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    nav_state_ = nav_state;
  }

  void updateImuBias(
    const gtsam::Key key, const double ts, const gtsam::imuBias::ConstantBias & imu_bias)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    imu_bias_ = imu_bias;
  }

  void updateNavStateAndImuBias(
    const gtsam::Key key, const double ts, const gtsam::NavState & nav_state,
    const gtsam::imuBias::ConstantBias & imu_bias)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    nav_state_ = nav_state;
    imu_bias_ = imu_bias;
  }

  void updateBaroBias(const gtsam::Key key, const double & ts, const double & baro_bias)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    baro_bias_ = baro_bias;
  }

  void updateExtrinsic(const gtsam::Key key, const double ts, const gtsam::Pose3 extrinsic)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    extrinsic_ = extrinsic;
  }

  void updateNavStateImuBiasAndBaroBias(
    const gtsam::Key & key, const double & ts, const gtsam::NavState & nav_state,
    const gtsam::imuBias::ConstantBias & imu_bias, const double & baro_bias)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_ = key;
    ts_ = ts;
    nav_state_ = nav_state;
    imu_bias_ = imu_bias;
    baro_bias_ = baro_bias;
  }
};
}  // namespace rig
