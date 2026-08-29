#include "rig/radar/mapper.hpp"

namespace rig
{
// TODO: move elsewhere
void targetsToPointcloud(const TargetVector & targets, pcl::PointCloud<radar::mmWavePoint> & cloud)
{
  cloud.points.clear();
  cloud.points.reserve(targets.size());
  for (const TargetData & t : targets) {
    radar::mmWavePoint p;
    p.x = t.x;
    p.y = t.y;
    p.z = t.z;
    p.velocity = t.radial_speed;
    p.intensity = t.intensity;

    cloud.points.emplace_back(p);
  }
}

Mapper::Mapper(const MapConfig & config) : config_(config)
{
  map_ = PointCloud::Ptr(new PointCloud);

  // TODO: use incremental voxl map instead?
  voxel_grid_.setLeafSize(
    config_.voxel_grid_leaf_size, config_.voxel_grid_leaf_size, config_.voxel_grid_leaf_size);
}

Mapper::~Mapper() {}

void Mapper::insert(const TargetVector & targets, const Eigen::Matrix4d & transform)
{
  StopWatch sw;

  PointCloud::Ptr cloud(new PointCloud);
  targetsToPointcloud(targets, *cloud);
  pcl::transformPointCloud(*cloud, *cloud, transform.cast<float>());

  buffer_.emplace(cloud);
  if (buffer_.size() > config_.n_scans_delay) {
    // add to map
    *map_ += *buffer_.front();

    // pop buffer front after used
    buffer_.pop();

    // filter map
    PointCloud::Ptr filtered(new PointCloud);
    voxel_grid_.setInputCloud(map_);
    voxel_grid_.filter(*filtered);

    map_ = filtered;
  }

#if VERBOSE
  std::cout << "\t\tinsert: " << sw.elapsed() << '\n';
#endif
}

bool Mapper::buildKdTree()
{
  StopWatch sw;
  if (map_->empty()) {
    return false;
  }
  // threading doesn't seem to help, according to hotspot
  kdtree_.reset(new nanoflann::KdTreeFLANN<PointType>(false, 25, 2));
  kdtree_->setInputCloud(map_);

#if VERBOSE
  std::cout << "\t\tkdtree: " << sw.elapsed() << '\n';
#endif

  return true;
}

bool Mapper::radiusSearch(
  const TargetData & target, const Eigen::Matrix4d & transform, Distribution & dist) const
{
  const Eigen::Vector3d temp =
    transform.block<3, 3>(0, 0) * Eigen::Vector3d(target.x, target.y, target.z) +
    transform.block<3, 1>(0, 3);
  radar::mmWavePoint point;
  point.getVector3fMap() = temp.cast<float>();

  std::vector<int> indices;
  std::vector<float> sq_dists;
  const int num_found = kdtree_->radiusSearch(point, config_.search_radius, indices, sq_dists);
  if (num_found < config_.minimum_neighbors) {
    return false;
  }

  Eigen::Vector3d sum_points = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_matrix = Eigen::Matrix3d::Zero();
  for (int i = 0; i < num_found; ++i) {
    const Eigen::Vector3d pt = map_->at(indices[i]).getVector3fMap().cast<double>();
    sum_points += pt;
    sum_matrix += pt * pt.transpose();
  }

  dist.mean = sum_points / num_found;
  dist.covariance = (sum_matrix - dist.mean * sum_points.transpose()) / (num_found - 1);

  return true;
}

}  // namespace rig
