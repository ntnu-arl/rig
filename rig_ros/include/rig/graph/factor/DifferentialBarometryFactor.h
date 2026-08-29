#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>

namespace rig
{
class DifferentialBarometryFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, double>
{
  double relative_height_;

public:
  typedef NoiseModelFactor2<gtsam::Pose3, double> Base;

  DifferentialBarometryFactor(
    const double & relative_height, const gtsam::Key & k0, const gtsam::Key & k1,
    const gtsam::SharedNoiseModel & model)
  : Base(model, k0, k1), relative_height_(relative_height)
  {
  }

  virtual ~DifferentialBarometryFactor() {}

  // Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const double & baro_bias,
    boost::optional<gtsam::Matrix &> H0 = boost::none,
    boost::optional<gtsam::Matrix &> H1 = boost::none) const override
  {
    const double estimated_height = pose_B_W.translation().z();
    const double measured_height = relative_height_ - baro_bias;

    const gtsam::Vector residual =
      (gtsam::Vector(1) << (estimated_height - measured_height)).finished();

    // df/dtau
    if (H0) {
      H0->resize(1, 6);

      // orientation
      (*H0).leftCols(3) = gtsam::Matrix13::Zero();
      // position
      // taking third row for z
      (*H0).rightCols(3) = pose_B_W.rotation().matrix().block<1, 3>(2, 0);
    }

    // df/dBaroBias
    if (H1) {
      H1->resize(1, 1);

      *H1 = gtsam::Matrix11::Ones();
    }

    return residual;
  }
};
}  // namespace rig
