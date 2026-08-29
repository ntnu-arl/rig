#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace rig
{
class VelocityFactor
: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
  gtsam::Vector3 linear_velocity_R_;  // radar velocity estimate from lst-sqr
  gtsam::Pose3 pose_R_B_;             // pose of radar in B
  gtsam::Vector3 angular_velocity_B_;

public:
  typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> Base;

  VelocityFactor(
    const gtsam::Vector3 & linear_velocity_R, const gtsam::Pose3 & pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key & k0, const gtsam::Key & k1,
    const gtsam::Key & k2, const gtsam::SharedNoiseModel & model)
  : Base(model, k0, k1, k2),
    linear_velocity_R_(linear_velocity_R),
    pose_R_B_(pose_R_B),
    angular_velocity_B_(angular_velocity_B)
  {
  }

  virtual ~VelocityFactor() {}

  // Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const gtsam::Vector3 & linear_velocity_W,
    const gtsam::imuBias::ConstantBias & imu_bias_B,
    boost::optional<gtsam::Matrix &> H0 = boost::none,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    const gtsam::Rot3 rot_R_B = pose_R_B_.rotation();
    const gtsam::Point3 l_R_B = pose_R_B_.translation();

    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();

    const gtsam::Vector3 linear_velocity_from_angular_B =
      (angular_velocity_B_ - imu_bias_B.gyroscope()).cross(l_R_B);
    const gtsam::Vector3 est_linear_velocity_R =
      rot_R_B.transpose() *
      (rot_B_W.transpose() * linear_velocity_W + linear_velocity_from_angular_B);

    const gtsam::Vector residual = est_linear_velocity_R - linear_velocity_R_;

    // df/dtau
    if (H0) {
      H0->resize(3, 6);

      // orientation
      (*H0).leftCols(3) =
        rot_R_B.transpose() * gtsam::skewSymmetric(rot_B_W.transpose() * linear_velocity_W);
      // position
      (*H0).rightCols(3) = gtsam::Matrix13::Zero();
    }

    // df/dv
    if (H1) {
      H1->resize(3, 3);

      *H1 = rot_R_B.transpose() * rot_B_W.transpose();
    }

    // df/dBias
    if (H2) {
      H2->resize(3, 6);

      // accelerometer
      (*H2).leftCols(3) = gtsam::Matrix13::Zero();
      // gyroscope
      (*H2).rightCols(3) = rot_R_B.transpose() * gtsam::skewSymmetric(l_R_B);
    }

    return residual;
  }
};
}  // namespace rig
