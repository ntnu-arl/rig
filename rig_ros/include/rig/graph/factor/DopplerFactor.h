#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace rig
{
class DopplerFactor
: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
  gtsam::Point3 point_measurement_;    // point in radar frame
  double doppler_measurement_;         // doppler velocity of point
  gtsam::Pose3 pose_R_B_;              // pose of radar in B
  gtsam::Vector3 angular_velocity_B_;  // angvel from IMU during radar exposure
public:
  typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> Base;

  DopplerFactor(
    const gtsam::Point3 & point, const double & doppler, const gtsam::Pose3 pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, gtsam::Key k0, gtsam::Key k1, gtsam::Key k2,
    const gtsam::SharedNoiseModel & model)
  : Base(model, k0, k1, k2),
    point_measurement_(point),
    doppler_measurement_(doppler),
    pose_R_B_(pose_R_B),
    angular_velocity_B_(angular_velocity_B)
  {
  }

  virtual ~DopplerFactor() {}

  // Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const gtsam::Vector3 & linear_velocity_W,
    const gtsam::imuBias::ConstantBias & imu_bias_B,
    boost::optional<gtsam::Matrix &> H0 = boost::none,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    const gtsam::Point3 point_hat = point_measurement_.normalized();  // normalize point
    const gtsam::Rot3 rot_R_B = pose_R_B_.rotation();                 // R from {R} to {B}
    const gtsam::Point3 l_R_B = pose_R_B_.translation();  // translation of {R} expressed in {B}
    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();      // R from {B} to {W}

    // calculate influence of angular velocity on linear velocity through l_R_B
    const gtsam::Vector3 linear_velocity_from_angular_B =
      (angular_velocity_B_ - imu_bias_B.gyroscope()).cross(l_R_B);
    const gtsam::Vector3 linear_velocity_R =
      rot_R_B.transpose() *
      (rot_B_W.transpose() * linear_velocity_W + linear_velocity_from_angular_B);

    // residual formulated as h(x) - z
    const double doppler_estimate = -point_hat.dot(linear_velocity_R);
    const gtsam::Vector residual =
      (gtsam::Vector(1) << (doppler_estimate - doppler_measurement_)).finished();

    // df/dtau
    if (H0) {
      H0->resize(1, 6);

      (*H0).leftCols(3) = -point_hat.transpose() * rot_R_B.transpose() *
                          (rot_B_W.transpose() * gtsam::skewSymmetric(linear_velocity_W) *
                           rot_B_W.matrix());        // rotation
      (*H0).rightCols(3) = gtsam::Matrix13::Zero();  // translation
    }

    // df/dv
    if (H1) {
      H1->resize(1, 3);

      *H1 = -point_hat.transpose() * rot_R_B.transpose() * rot_B_W.transpose();
    }

    // df/dBias
    if (H2) {
      H2->resize(1, 6);

      (*H2).leftCols(3) = gtsam::Matrix13::Zero();  // accelerometer
      (*H2).rightCols(3) =
        -point_hat.transpose() * rot_R_B.transpose() * gtsam::skewSymmetric(l_R_B);  // gyroscope
    }

    return residual;
  }
};
}  // namespace rig
