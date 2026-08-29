#pragma once

#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include "rig/radar/point.hpp"

namespace rig
{
struct Distribution
{
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;

  Distribution(
    const Eigen::Vector3d & mean = Eigen::Vector3d::Zero(),
    const Eigen::Matrix3d & covariance = Eigen::Matrix3d::Identity())
  : mean(mean), covariance(covariance)
  {
  }

  template <typename PointT>
  Distribution(const pcl::PointCloud<PointT> & cloud, const std::vector<int> & indices)
  {
    const size_t N = indices.size();
    Eigen::Vector3d sum_points = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_matrix = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      const auto & point = cloud.at(indices[i]).getVector3fMap().template cast<double>();
      sum_points += point;
      sum_matrix += point * point.transpose();
    }

    mean = sum_points / N;
    covariance = (sum_matrix - mean * sum_points.transpose()) / (N - 1);
  }
};

typedef std::vector<Distribution> DistributionVector;
}  // namespace rig
