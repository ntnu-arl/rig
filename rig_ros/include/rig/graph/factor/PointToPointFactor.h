#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "rig/radar/distribution.hpp"
#include "rig/radar/mapper.hpp"
#include "rig/radar/target_data.hpp"

namespace rig
{
class PointToPointFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
  gtsam::Point3 point_R_;  // point in radar frame
  gtsam::Point3 mean_W_;   // mean of Gaussian in W
public:
  typedef NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;

  PointToPointFactor(
    const gtsam::Point3 & point_R, const gtsam::Point3 & mean_W, gtsam::Key k0, gtsam::Key k1,
    const gtsam::SharedNoiseModel & model)
  : Base(model, k0, k1), point_R_(point_R), mean_W_(mean_W)
  {
  }

  virtual ~PointToPointFactor() {}

  // Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const gtsam::Pose3 & pose_R_B,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    gtsam::Matrix H_compose1, H_compose2;
    const gtsam::Pose3 pose_R_W =
      pose_B_W.compose(pose_R_B, H_compose1, H_compose2);  // transform from {R} to {W}

    // residual formulated as h(x) - z
    gtsam::Matrix H_transform;
    gtsam::Vector residual =
      gtsam::Vector3(pose_R_W.transformFrom(point_R_, H_transform) - mean_W_);

    if (H1) {
      *H1 = H_transform * H_compose1;
    }
    if (H2) {
      *H2 = H_transform * H_compose2;
    }

    return residual;
  }
};

class PointToDistributionHessianFactor : public gtsam::NonlinearFactor
{
  typedef gtsam::Matrix66 M66;
  typedef gtsam::Vector6 V6;

  typedef gtsam::noiseModel::mEstimator::Base RobustModel;

private:
  TargetVector targets_;                // targets in radar frame
  std::shared_ptr<Mapper> mapper_ptr_;  // kdtree object
  double range_sigma_;                  // in m
  double phase_sigma_;                  // in rad

  mutable TargetVector matched_targets_;
  mutable DistributionVector matched_features_;

  RobustModel::shared_ptr robust_;

  mutable std::shared_ptr<gtsam::GaussianFactor> gaussian_factor_ = nullptr;

  mutable size_t num_active_factors_;

  typedef gtsam::NonlinearFactor Base;

public:
  PointToDistributionHessianFactor(
    const TargetVector & targets, std::shared_ptr<Mapper> mapper_ptr, const gtsam::Key key0,
    const gtsam::Key key1, const double range_sigma, const double phase_sigma,
    const RobustModel::shared_ptr & robust)
  : Base(std::vector<gtsam::Key>{key0, key1}),
    targets_(targets),
    mapper_ptr_(mapper_ptr),
    range_sigma_(range_sigma),
    phase_sigma_(phase_sigma),
    robust_(robust)
  {
    matched_targets_.reserve(targets_.size());
    matched_features_.reserve(targets_.size());
  }

  ~PointToDistributionHessianFactor() override {}

  const TargetVector & getTargets() const { return matched_targets_; }
  const DistributionVector & getFeatures() const { return matched_features_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PointToDistributionHessianFactor(*this)));
  }

  size_t dim() const override
  {
    return 12;  // The Hessian is 12x12
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
    const gtsam::Pose3 pose_B_W = c.at<gtsam::Pose3>(keys()[0]);  // pose of {B} in {W}
    const gtsam::Pose3 pose_R_B = c.at<gtsam::Pose3>(keys()[1]);  // pose of {R} in {B}

    // pre-calculate non-measurement dependent jacobians
    gtsam::Matrix j1, j2;
    const gtsam::Pose3 pose_R_W = pose_B_W.compose(pose_R_B, j1, j2);  // transform from {R} to {W}

    // noises
    // TODO: cache
    const double range_covar = std::pow(range_sigma_, 2);
    const gtsam::Matrix22 phase_covar = gtsam::Matrix22::Identity() * std::pow(phase_sigma_, 2);

    // find associations (can be slow)
    matched_targets_.clear();
    matched_features_.clear();
    Distribution dist;
    for (const auto & target : targets_) {
      if (mapper_ptr_->radiusSearch(target, pose_R_W.matrix(), dist)) {
        matched_targets_.push_back(target);
        matched_features_.push_back(dist);
      }
    }

    // pose [rotation, translation]
    M66 G11 = M66::Zero();
    M66 G12 = M66::Zero();
    V6 g1 = V6::Zero();
    // extrinsic [rotation, translation]
    M66 G22 = M66::Zero();
    V6 g2 = V6::Zero();
    // f
    double f = 0.0;

    const size_t N = matched_targets_.size();
    for (size_t i = 0; i < N; ++i) {
      const gtsam::Point3 point =
        gtsam::Point3(matched_targets_[i].x, matched_targets_[i].y, matched_targets_[i].z);

      gtsam::Matrix H_transform;
      // compute the error
      const gtsam::Vector e =
        pose_R_W.transformFrom(point, H_transform) - matched_features_[i].mean;

      // Compute the Jacobians
      // pose [rotation, position]
      const gtsam::Matrix36 J1 = H_transform * j1;
      // extrinsic [rotation, position]
      const gtsam::Matrix36 J2 = H_transform * j2;

      // calculate covariance
      // TODO: use noise model
      const double range = matched_targets_[i].range;
      const gtsam::Vector3 bearing = point / range;
      const gtsam::Matrix33 point_covar =
        range_covar * bearing * bearing.transpose() +
        range * range * matched_targets_[i].bearingCovariance(phase_covar);
      const gtsam::Matrix33 point_covar_trans =
        pose_R_W.rotation().matrix() * point_covar * pose_R_W.rotation().transpose();
      // TODO: cache noise model for efficiency
      auto noise_model = gtsam::noiseModel::Gaussian::Covariance(
        point_covar_trans + matched_features_[i].covariance);

      // Set whitened errors and jacobians
      gtsam::Vector e_whitened = noise_model->whiten(e);
      gtsam::Matrix J1_whitened = noise_model->Whiten(J1);
      gtsam::Matrix J2_whitened = noise_model->Whiten(J2);

      // apply robust cost function
      robust_->reweight(J1_whitened, J2_whitened, e_whitened);

      // TODO: optimize matrix transpose multiplication?
      G11 += J1_whitened.transpose() * J1_whitened;
      G12 += J1_whitened.transpose() * J2_whitened;
      g1 += -J1_whitened.transpose() * e_whitened;

      G22 += J2_whitened.transpose() * J2_whitened;
      g2 += -J2_whitened.transpose() * e_whitened;

      f += e_whitened.transpose() * e_whitened;
    }

    // Create the factor
    const std::vector<gtsam::Matrix> matrices = {G11, G12, G22};
    const std::vector<gtsam::Vector> vectors = {g1, g2};
    return boost::make_shared<gtsam::HessianFactor>(keys(), matrices, vectors, f);
  }
};
}  // namespace rig
