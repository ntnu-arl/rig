#pragma once

#include <pcl/point_types.h>

namespace rig
{
namespace radar
{
struct rioPoint
{
  PCL_ADD_POINT4D;      // position [m]
  float snr_db;         // CFAR cell to side noise ratio [dB]
  float noise_db;       // CFAR noise level of the side of the detected cell [dB]
  float v_doppler_mps;  // Doppler speed [m/s]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct mmWavePoint
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // RCS?
  float velocity;   // Doppler speed [m/s]
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct mmWavePointExtra
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // snr
  float velocity;   // Doppler speed [m/s]
  float rcs;        // rcs estimate from snr and range
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct mmWaveDopplerResidualPoint
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // RCS?
  float velocity;   // Doppler speed [m/s]
  float doppler_residual;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace radar
}  // namespace rig

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  rig::radar::rioPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, snr_db, snr_db)
  (float, noise_db, noise_db)
  (float, v_doppler_mps, v_doppler_mps)
)
// clang-format on
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  rig::radar::mmWavePoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, velocity, velocity)
)
// clang-format on
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  rig::radar::mmWavePointExtra,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, velocity, velocity)
  (float, rcs, rcs)
)
// clang-format on
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  rig::radar::mmWaveDopplerResidualPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, velocity, velocity)
  (float, doppler_residual, doppler_residual)
)
// clang-format on
