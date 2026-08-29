#pragma once

// GTSAM
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "rig/radar/target_data.hpp"

namespace rig
{
class DopplerHessianFactor : public gtsam::NonlinearFactor
{
public:
  enum Status { Static = 0, Dynamic };

  const std::vector<Status> & getStatus() const { return target_status_; }
  const TargetVector & getStatic() const { return static_targets_; }
  const TargetVector & getDynamic() const { return dynamic_targets_; }

private:
  /* mutable */ TargetVector targets_;  // targets in radar frame
  gtsam::Pose3 pose_R_B_;               // pose of radar in B
  gtsam::Vector3 angular_velocity_B_;   // angvel from IMU during radar exposure;

  double noise_sigma_;      // in m/s
  double huber_threshold_;  // in std deviations

  mutable std::vector<Status> target_status_;  // classification of targets as static or non-static
  mutable TargetVector static_targets_;
  mutable TargetVector dynamic_targets_;

  mutable std::shared_ptr<gtsam::GaussianFactor> gaussian_factor_ = nullptr;

  mutable size_t num_active_factors_;

  typedef gtsam::NonlinearFactor Base;

public:
  DopplerHessianFactor(
    const TargetVector & targets, const gtsam::Pose3 & pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key key0, const gtsam::Key key1,
    const gtsam::Key key2, const double noise_sigma, const double huber_threshold)
  : Base(std::vector<gtsam::Key>{key0, key1, key2}),
    targets_(targets),
    pose_R_B_(pose_R_B),
    angular_velocity_B_(angular_velocity_B),
    noise_sigma_(noise_sigma),
    huber_threshold_(huber_threshold)
  {
    target_status_.resize(targets_.size());
  }

  ~DopplerHessianFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DopplerHessianFactor(*this)));
  }

  size_t dim() const override
  {
    return 15;  // The Hessian is 15x15
  }

  double error([[maybe_unused]] const gtsam::Values & c) const override
  {
    std::cout << "\033[1;31mCalling Error on Key: \033[0m\n"
              << gtsam::DefaultKeyFormatter(keys()[0]) << "\n";
    return 0.0;
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & c) const override
  {
    const gtsam::Rot3 rot_R_B = pose_R_B_.rotation();     // R from {R} to {B}
    const gtsam::Point3 l_R_B = pose_R_B_.translation();  // translation of {R} expressed in {B}
    const gtsam::Pose3 pose_B_W = c.at<gtsam::Pose3>(keys()[0]);  // pose of {B} wrt {W}
    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();              // R from {B} to {W}
    const gtsam::Vector3 linear_velocity_W =
      c.at<gtsam::Vector3>(keys()[1]);  // linear velocity in {W}
    const gtsam::imuBias::ConstantBias imu_bias_B =
      c.at<gtsam::imuBias::ConstantBias>(keys()[2]);  // imu bias

    // calculate influence of angular velocity on linear velocity through l_R_B
    const gtsam::Vector3 linear_velocity_from_angular_B =
      (angular_velocity_B_ - imu_bias_B.gyroscope()).cross(l_R_B);
    const gtsam::Vector3 linear_velocity_R =
      rot_R_B.transpose() *
      (rot_B_W.transpose() * linear_velocity_W + linear_velocity_from_angular_B);

    gtsam::Matrix66 G11 = gtsam::Matrix66::Zero();
    gtsam::Matrix63 G12 = gtsam::Matrix63::Zero();
    gtsam::Matrix66 G13 = gtsam::Matrix66::Zero();
    gtsam::Matrix33 G22 = gtsam::Matrix33::Zero();
    gtsam::Matrix36 G23 = gtsam::Matrix36::Zero();
    gtsam::Matrix66 G33 = gtsam::Matrix66::Zero();
    gtsam::Vector6 g1 = gtsam::Vector6::Zero();
    gtsam::Vector3 g2 = gtsam::Vector3::Zero();
    gtsam::Vector6 g3 = gtsam::Vector6::Zero();
    double f = 0.0;

    int i = -1;
    static_targets_.clear();
    static_targets_.reserve(targets_.size());
    dynamic_targets_.clear();
    dynamic_targets_.reserve(targets_.size());
    for (const TargetData & t : targets_) {
      i++;

      const gtsam::Point3 bearing = gtsam::Point3(t.x, t.y, t.z) / t.range;
      const double doppler = t.radial_speed;

      // Compute the error
      const double e = -bearing.dot(linear_velocity_R) - doppler;

      // Compute the Jacobians
      gtsam::Matrix16 J1;
      J1.leftCols(3) = -bearing.transpose() * rot_R_B.transpose() *
                       (rot_B_W.transpose() * gtsam::skewSymmetric(linear_velocity_W) *
                        rot_B_W.matrix());        // rotation;
      J1.rightCols(3) = gtsam::Matrix13::Zero();  // translation;
      gtsam::Matrix13 J2 =
        -bearing.transpose() * rot_R_B.transpose() * rot_B_W.transpose();  // velocity
      gtsam::Matrix16 J3;
      J3.leftCols(3) = gtsam::Matrix13::Zero();  // accelerometer;
      J3.rightCols(3) =
        -bearing.transpose() * rot_R_B.transpose() * gtsam::skewSymmetric(l_R_B);  // gyroscope

      // Set whitened errors
      gtsam::Matrix J1_whitened = J1 / noise_sigma_;
      gtsam::Matrix J2_whitened = J2 / noise_sigma_;
      gtsam::Matrix J3_whitened = J3 / noise_sigma_;
      double e_whitened = e / noise_sigma_;

      // Apply Robust cost
      const double weight =
        abs(e_whitened) <= huber_threshold_ ? 1.0 : std::sqrt(huber_threshold_ / abs(e_whitened));

      J1_whitened *= weight;
      J2_whitened *= weight;
      J3_whitened *= weight;
      e_whitened *= weight;

      G11 += J1_whitened.transpose() * J1_whitened;
      G12 += J1_whitened.transpose() * J2_whitened;
      G13 += J1_whitened.transpose() * J3_whitened;
      G22 += J2_whitened.transpose() * J2_whitened;
      G23 += J2_whitened.transpose() * J3_whitened;
      G33 += J3_whitened.transpose() * J3_whitened;
      g1 += -J1_whitened.transpose() * e_whitened;
      g2 += -J2_whitened.transpose() * e_whitened;
      g3 += -J3_whitened.transpose() * e_whitened;
      f += e_whitened * e_whitened;
    }

    // Create the factor
    return boost::make_shared<gtsam::HessianFactor>(
      keys()[0], keys()[1], keys()[2], G11, G12, G13, g1, G22, G23, g2, G33, g3, f);
  }
};

class DopplerExtrinsicHessianFactor : public gtsam::NonlinearFactor
{
  typedef gtsam::Matrix33 M33;
  typedef gtsam::Matrix36 M36;
  typedef gtsam::Matrix63 M63;
  typedef gtsam::Matrix66 M66;
  typedef gtsam::Vector3 V3;
  typedef gtsam::Vector6 V6;

  typedef gtsam::noiseModel::mEstimator::Base RobustModel;

public:
  enum Status { Static = 0, Dynamic };

  const std::vector<Status> & getStatus() const { return target_status_; }
  const TargetVector & getStatic() const { return static_targets_; }
  const TargetVector & getDynamic() const { return dynamic_targets_; }

  void updateStaticDynamic()
  {
    static_targets_.clear();
    static_targets_.reserve(targets_.size());
    dynamic_targets_.clear();
    dynamic_targets_.reserve(targets_.size());
    // fill static/dynamic vectors
    for (size_t i = 0; i < targets_.size(); ++i) {
      const Status ts = target_status_[i];
      if (ts == Static) {
        static_targets_.push_back(targets_[i]);
      } else if (ts == Dynamic) {
        dynamic_targets_.push_back(targets_[i]);
      }
    }
  }

  enum AliasStatus { Lesser = 0, Greater };
  const std::vector<AliasStatus> & getAliasStatus() const { return alias_status_; }
  const TargetVector & getLesser() const { return lesser_targets_; }
  const TargetVector & getGreater() const { return greater_targets_; }

  void updateAliasTargets()
  {
    lesser_targets_.clear();
    lesser_targets_.reserve(targets_.size());
    greater_targets_.clear();
    greater_targets_.reserve(targets_.size());
    // fill static/dynamic vectors
    for (size_t i = 0; i < targets_.size(); ++i) {
      const AliasStatus ts = alias_status_[i];
      if (ts == Lesser) {
        lesser_targets_.push_back(targets_[i]);
      } else if (ts == Greater) {
        greater_targets_.push_back(targets_[i]);
      }
    }
  }

  gtsam::Vector3 radarVelocity(const gtsam::Values & c) const
  {
// extracting from keys
#if USE_POS
    const gtsam::Rot3 rot_B_W = c.at<gtsam::Pose3>(keys()[0]).rotation();  // R from {B} to {W}
#else
    const gtsam::Rot3 rot_B_W = c.at<gtsam::Rot3>(keys()[0]);  // R from {B} to {W}
#endif
    const gtsam::Vector3 linear_velocity_W =
      c.at<gtsam::Vector3>(keys()[1]);  // linear velocity in {W}
    const gtsam::imuBias::ConstantBias imu_bias_B =
      c.at<gtsam::imuBias::ConstantBias>(keys()[2]);              // imu bias
    const gtsam::Pose3 pose_R_B = c.at<gtsam::Pose3>(keys()[3]);  // pose of {R} in {B}
    // intermediate variables
    const gtsam::Rot3 rot_R_B = pose_R_B.rotation();      // R from {R} to {B}
    const gtsam::Point3 l_BR_B = pose_R_B.translation();  // translation of {R} expressed in {B}

    // calculate influence of angular velocity on linear velocity through l_BR_B
    const gtsam::Vector3 linear_velocity_B = rot_B_W.transpose() * linear_velocity_W;
    const gtsam::Vector3 linear_velocity_from_angular_B =
      imu_bias_B.correctGyroscope(angular_velocity_B_).cross(l_BR_B);

    return rot_R_B.transpose() * (linear_velocity_B + linear_velocity_from_angular_B);
  }

private:
  /* mutable */ TargetVector targets_;  // targets in radar frame
  gtsam::Vector3 angular_velocity_B_;   // angvel from IMU during radar exposure;

  double doppler_sigma_;  // in m/s
  double phase_sigma_;    // in rad
  double doppler_max_;
  RobustModel::shared_ptr robust_less_;     // <= doppler max
  RobustModel::shared_ptr robust_greater_;  // > doppler max
  bool estimate_extrinsic_;

  mutable std::vector<Status> target_status_;  // classification of targets as static or non-static
  mutable TargetVector static_targets_;
  mutable TargetVector dynamic_targets_;
  mutable std::vector<AliasStatus>
    alias_status_;  // classification of targets as lesser or greater than doppler max
  mutable TargetVector lesser_targets_;
  mutable TargetVector greater_targets_;

  mutable std::shared_ptr<gtsam::GaussianFactor> gaussian_factor_ = nullptr;

  mutable size_t num_active_factors_;

  typedef gtsam::NonlinearFactor Base;

public:
  DopplerExtrinsicHessianFactor(
    const TargetVector & targets, const gtsam::Vector3 & angular_velocity_B, const gtsam::Key key0,
    const gtsam::Key key1, const gtsam::Key key2, const gtsam::Key key3, const double doppler_sigma,
    const double phase_sigma, const double doppler_max, const RobustModel::shared_ptr & robust_less,
    const RobustModel::shared_ptr & robust_greater, const bool estimate_extrinsic = true)
  : Base(std::vector<gtsam::Key>{key0, key1, key2, key3}),
    targets_(targets),
    angular_velocity_B_(angular_velocity_B),
    doppler_sigma_(doppler_sigma),
    phase_sigma_(phase_sigma),
    doppler_max_(doppler_max),
    robust_less_(robust_less),
    robust_greater_(robust_greater),
    estimate_extrinsic_(estimate_extrinsic)
  {
    target_status_.resize(targets_.size());
    static_targets_.reserve(targets_.size());

    alias_status_.resize(targets_.size());
    lesser_targets_.reserve(targets_.size());
    greater_targets_.reserve(targets_.size());
  }

  ~DopplerExtrinsicHessianFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DopplerExtrinsicHessianFactor(*this)));
  }

  size_t dim() const override
  {
    return 15;  // The Hessian is 15x15
  }

  double error([[maybe_unused]] const gtsam::Values & c) const override
  {
    std::cout << "\033[1;31mCalling Error on Key: \033[0m\n"
              << gtsam::DefaultKeyFormatter(keys()[0]) << "\n";
    return 0.0;
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & c) const override
  {
// extracting from keys
#if USE_POS
    const gtsam::Rot3 rot_B_W = c.at<gtsam::Pose3>(keys()[0]).rotation();  // R from {B} to {W}
#else
    const gtsam::Rot3 rot_B_W = c.at<gtsam::Rot3>(keys()[0]);  // R from {B} to {W}
#endif
    const gtsam::Vector3 linear_velocity_W =
      c.at<gtsam::Vector3>(keys()[1]);  // linear velocity in {W}
    const gtsam::imuBias::ConstantBias imu_bias_B =
      c.at<gtsam::imuBias::ConstantBias>(keys()[2]);              // imu bias
    const gtsam::Pose3 pose_R_B = c.at<gtsam::Pose3>(keys()[3]);  // pose of {R} in {B}
    const gtsam::Rot3 rot_R_B = pose_R_B.rotation();              // R from {R} to {B}
    const gtsam::Point3 l_BR_B = pose_R_B.translation();  // translation of {R} expressed in {B}
    const gtsam::Vector3 linear_velocity_B = rot_B_W.transpose() * linear_velocity_W;

    const gtsam::Vector3 linear_velocity_R = radarVelocity(c);

    // pre-calculate non-measurement dependent jacobians
    const gtsam::Matrix33 j1_l = rot_R_B.transpose() * gtsam::skewSymmetric(linear_velocity_B);
    const gtsam::Matrix33 j2 = rot_R_B.transpose() * rot_B_W.transpose();
    const gtsam::Matrix33 j3_r = rot_R_B.transpose() * gtsam::skewSymmetric(l_BR_B);
    gtsam::Matrix33 j4_l, j4_r;
    if (estimate_extrinsic_) {
      j4_l = gtsam::skewSymmetric(linear_velocity_R);
      j4_r = gtsam::skewSymmetric(
        rot_R_B.transpose() * imu_bias_B.correctGyroscope(angular_velocity_B_));
    } else {
      j4_l = j4_r = gtsam::Matrix33::Zero();
    }

    const double doppler_covar = std::pow(doppler_sigma_, 2);
    const gtsam::Matrix22 phase_covar = gtsam::Matrix22::Identity() * std::pow(phase_sigma_, 2);

    static_targets_.clear();

// pose/rotation
#if USE_POS
    gtsam::Matrix66 G11 = gtsam::Matrix66::Zero();
    gtsam::Matrix63 G12 = gtsam::Matrix63::Zero();
    gtsam::Matrix66 G13 = gtsam::Matrix66::Zero();
    gtsam::Matrix66 G14 = gtsam::Matrix66::Zero();
    gtsam::Vector6 g1 = gtsam::Vector6::Zero();
#else
    gtsam::Matrix33 G11 = gtsam::Matrix33::Zero();
    gtsam::Matrix33 G12 = gtsam::Matrix33::Zero();
    gtsam::Matrix36 G13 = gtsam::Matrix36::Zero();
    gtsam::Matrix36 G14 = gtsam::Matrix36::Zero();
    gtsam::Vector3 g1 = gtsam::Vector3::Zero();
#endif
    // velocity
    gtsam::Matrix33 G22 = gtsam::Matrix33::Zero();
    gtsam::Matrix36 G23 = gtsam::Matrix36::Zero();
    gtsam::Matrix36 G24 = gtsam::Matrix36::Zero();
    gtsam::Vector3 g2 = gtsam::Vector3::Zero();
    // bias
    gtsam::Matrix66 G33 = gtsam::Matrix66::Zero();
    gtsam::Matrix66 G34 = gtsam::Matrix66::Zero();
    gtsam::Vector6 g3 = gtsam::Vector6::Zero();
    // extrinsic
    gtsam::Matrix66 G44 = gtsam::Matrix66::Zero();
    gtsam::Vector6 g4 = gtsam::Vector6::Zero();
    // f
    double f = 0.0;

    int i = -1;
    for (const TargetData & t : targets_) {
      i++;

      const gtsam::Point3 bearing = gtsam::Point3(t.x, t.y, t.z) / t.range;
      const double doppler_measurement = t.radial_speed;

      // Compute the error
      const double doppler_prediction = -bearing.dot(linear_velocity_R);
      const double e = doppler_prediction - doppler_measurement;

      if (std::abs(doppler_prediction) > doppler_max_) {
        alias_status_[i] = Greater;
      } else {
        alias_status_[i] = Lesser;
      }

// Compute the Jacobians
#if USE_POS
      gtsam::Matrix16 J1;  // rotation
      J1.leftCols(3) = -bearing.transpose() * j1_l;
      J1.rightCols(3) = gtsam::Matrix13::Zero();
#else
      gtsam::Matrix13 J1;  // rotation
      J1 = -bearing.transpose() * j1_l;
#endif
      gtsam::Matrix13 J2;  // velocity
      J2 = -bearing.transpose() * j2;
      gtsam::Matrix16 J3;  // bias [accelerometer, gyroscope]
      J3.leftCols(3) = gtsam::Matrix13::Zero();
      J3.rightCols(3) = -bearing.transpose() * j3_r;
      gtsam::Matrix16 J4;  // extrinsic [rotation, translation]
      J4.leftCols(3) = -bearing.transpose() * j4_l;
      J4.rightCols(3) = -bearing.transpose() * j4_r;

      // calculate covariance
      // cosnt double covar = doppler_covar + ;
      const double temp =
        linear_velocity_R.transpose() * t.bearingCovariance(phase_covar) * linear_velocity_R;
      const double sigma = std::sqrt(doppler_covar + temp);
      // std::cout << angles::to_degrees(t.azimuth) << '\t' << angles::to_degrees(t.elevation) << '\n';
      // std::cout << angles::to_degrees(t.wy) << '\t' << angles::to_degrees(t.wz) << '\n';
      // std::cout << doppler_covar << '\t' << phase_covar.diagonal().transpose() << '\t' << temp << '\n';
      // std::cout << '\n';
      // const double sigma = std::sqrt(doppler_covar);

      // std::cout << "std: " << doppler_sigma_ << '\t' << std::sqrt(doppler_covar + temp) << '\t'
      //           << std::sqrt(doppler_covar - temp) << '\n';

      // Set whitened errors
      double e_whitened = e / sigma;

      // TODO: parametrize
      if (e_whitened < 3.0) {  // standard deviations
        static_targets_.emplace_back(t);
      }

      // Apply Robust cost
      // NOTE: very important that it's sqrt-weight
      const auto & robust =
        (std::abs(doppler_prediction) > doppler_max_) ? robust_greater_ : robust_less_;
      const double sqrt_weight = std::sqrt(robust->weight(e_whitened));

      e_whitened *= sqrt_weight;
      // apply weighting to jacobians in one go
      const double scaling = sqrt_weight / sigma;
      const gtsam::Matrix J1_whitened = J1 * scaling;
      const gtsam::Matrix J2_whitened = J2 * scaling;
      const gtsam::Matrix J3_whitened = J3 * scaling;
      const gtsam::Matrix J4_whitened = J4 * scaling;

      // TODO: optimize matrix transpose multiplication?
      G11 += J1_whitened.transpose() * J1_whitened;
      G12 += J1_whitened.transpose() * J2_whitened;
      G13 += J1_whitened.transpose() * J3_whitened;
      G14 += J1_whitened.transpose() * J4_whitened;
      g1 += -J1_whitened.transpose() * e_whitened;

      G22 += J2_whitened.transpose() * J2_whitened;
      G23 += J2_whitened.transpose() * J3_whitened;
      G24 += J2_whitened.transpose() * J4_whitened;
      g2 += -J2_whitened.transpose() * e_whitened;

      G33 += J3_whitened.transpose() * J3_whitened;
      G34 += J3_whitened.transpose() * J4_whitened;
      g3 += -J3_whitened.transpose() * e_whitened;

      G44 += J4_whitened.transpose() * J4_whitened;
      g4 += -J4_whitened.transpose() * e_whitened;

      f += e_whitened * e_whitened;
    }

    // Create the factor
    const std::vector<gtsam::Matrix> matrices = {G11, G12, G13, G14, G22, G23, G24, G33, G34, G44};
    const std::vector<gtsam::Vector> vectors = {g1, g2, g3, g4};
    return boost::make_shared<gtsam::HessianFactor>(keys(), matrices, vectors, f);
  }
};
}  // namespace rig
