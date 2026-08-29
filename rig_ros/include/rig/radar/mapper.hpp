#pragma once

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <queue>
#include <tuple>

#include <Eigen/Geometry>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rig/common/config.hpp"
#include "rig/common/stopwatch.hpp"
#include "rig/radar/distribution.hpp"
#include "rig/radar/point.hpp"
#include "rig/radar/target_data.hpp"
#include "thirdparty/nanoflann/nanoflann_pcl.hpp"

namespace rig
{
class Mapper
{
#define VERBOSE false

public:
  typedef radar::mmWavePoint PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

public:
  Mapper(const MapConfig & config);
  ~Mapper();

  void insert(const TargetVector & targets, const Eigen::Matrix4d & transform);
  bool buildKdTree();
  bool radiusSearch(
    const TargetData & target, const Eigen::Matrix4d & transform, Distribution & dist) const;

  const PointCloud::Ptr getMap() const { return map_; }

private:
  // parameters
  const MapConfig config_;

  // for matching
  std::unique_ptr<nanoflann::KdTreeFLANN<PointType>> kdtree_;

  // for mapping
  pcl::VoxelGrid<PointType> voxel_grid_;
  PointCloud::Ptr map_;
  std::queue<PointCloud::Ptr> buffer_;
};
}  // namespace rig
