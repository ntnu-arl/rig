#pragma once

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/publisher.h>
#include <tf2/LinearMath/Transform.h>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

// Eigen
#include <Eigen/Geometry>

namespace rig
{
inline void Convert(const tf2::Transform & t, geometry_msgs::Pose & p)
{
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  p.orientation.w = t.getRotation().w();
  p.orientation.x = t.getRotation().x();
  p.orientation.y = t.getRotation().y();
  p.orientation.z = t.getRotation().z();
}

inline void Convert(const tf2::Transform & t, nav_msgs::Odometry & odom)
{
  Convert(t, odom.pose.pose);
}

inline void Convert(const Eigen::Matrix4d & m, gtsam::Pose3 & p)
{
  p = gtsam::Pose3(gtsam::Rot3(m.block<3, 3>(0, 0)), gtsam::Point3(m(0, 3), m(1, 3), m(2, 3)));
}

inline void Convert(const tf2::Transform & t, geometry_msgs::TransformStamped & tfs)
{
  tfs.transform.translation.x = t.getOrigin().x();
  tfs.transform.translation.y = t.getOrigin().y();
  tfs.transform.translation.z = t.getOrigin().z();
  tfs.transform.rotation.x = t.getRotation().x();
  tfs.transform.rotation.y = t.getRotation().y();
  tfs.transform.rotation.z = t.getRotation().z();
  tfs.transform.rotation.w = t.getRotation().w();
}

inline void Convert(const Eigen::Vector3d & e_vec, gtsam::Vector3 & g_vec) { g_vec = e_vec; }

inline void Convert(const gtsam::Vector3 & g_vec, geometry_msgs::Vector3 & r_vec)
{
  r_vec.x = g_vec.x();
  r_vec.y = g_vec.y();
  r_vec.z = g_vec.z();
}

inline void Convert(const gtsam::Pose3 & p, tf2::Transform & t)
{
  t.setOrigin(tf2::Vector3(p.translation().x(), p.translation().y(), p.translation().z()));
  auto q = p.rotation().toQuaternion();
  t.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
}

inline void Convert(const gtsam::Pose3 & p, geometry_msgs::Pose & gmp)
{
  auto q = p.rotation().toQuaternion();
  gmp.orientation.w = q.w();
  gmp.orientation.x = q.x();
  gmp.orientation.y = q.y();
  gmp.orientation.z = q.z();
  gmp.position.x = p.translation().x();
  gmp.position.y = p.translation().y();
  gmp.position.z = p.translation().z();
}

inline void PublishOdometry(
  ros::Publisher & pub, const std::string & frame_id, const std::string & child_frame_id,
  const ros::Time & stamp, const tf2::Transform & t, const gtsam::Velocity3 & v,
  const gtsam::Matrix & pose_covar, const gtsam::Matrix & vel_covar)
{
  if (pub.getNumSubscribers()) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    Convert(t, odom);
#if USE_POS
    // pose covar
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        odom.pose.covariance[6 * i + j] = pose_covar(i, j);
      }
    }
#else
    // rot only covar
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        odom.pose.covariance[6 * (i + 3) + (j + 3)] = pose_covar(i, j);
      }
    }
#endif
    Convert(v, odom.twist.twist.linear);
    // vel only covar
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        odom.twist.covariance[6 * i + j] = vel_covar(i, j);
      }
    }
    pub.publish(odom);
  }
}

inline void PublishPath(ros::Publisher & pub, const ros::Time & stamp, nav_msgs::Path & path)
{
  if (pub.getNumSubscribers()) {
    path.header.stamp = stamp;
    pub.publish(path);
  }
}

inline void PublishPath(
  ros::Publisher & pub, const ros::Time & stamp, const tf2::Transform & t, nav_msgs::Path & path)
{
  if (pub.getNumSubscribers()) {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = stamp;
    Convert(t, ps.pose);
    path.poses.push_back(ps);
    PublishPath(pub, stamp, path);
  }
}

inline float Degrees(const float radians) { return radians * 180.0 / M_PI; }

}  // namespace rig
