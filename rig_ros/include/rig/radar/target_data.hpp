#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace rig
{
struct TargetData
{
  double x;
  double y;
  double z;
  double range;
  double azimuth;
  double elevation;
  double radial_speed;
  double intensity;
  // for noise modeling
  double wy;
  double wz;
  Eigen::Matrix<double, 3, 2> J_bearing;

  TargetData(
    const double x, const double y, const double z, const double range, const double azimuth,
    const double elevation, const double radial_speed, const double intensity)
  : x(x),
    y(y),
    z(z),
    range(range),
    azimuth(azimuth),
    elevation(elevation),
    radial_speed(radial_speed),
    intensity(intensity)
  {
    this->wy = M_PI * std::sin(azimuth) * std::cos(elevation);
    this->wz = M_PI * std::sin(elevation);

    this->J_bearing.setZero();
    const double denom =
      std::pow(M_PI, 2) *
      std::sqrt(1.0 - ((std::pow(this->wy, 2) + std::pow(this->wz, 2)) / std::pow(M_PI, 2)));

    this->J_bearing(0, 0) = -this->wy / denom;
    this->J_bearing(0, 1) = -this->wz / denom;
    this->J_bearing(1, 0) = 1 / M_PI;
    this->J_bearing(2, 1) = 1 / M_PI;
  }

  // TODO: can be cached
  Eigen::Matrix3d bearingCovariance(const Eigen::Matrix2d & phase_covar) const
  {
    return J_bearing * phase_covar * J_bearing.transpose();
  }
};

typedef std::vector<TargetData> TargetVector;
}  // namespace rig
