#include "rig/graph/graph_manager.hpp"
#include "rig/imu/imu_manager.hpp"
#include "rig/radar/radar_manager.hpp"

// rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

// C++
#include <filesystem>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rig_node");
  ros::NodeHandle pnh("~");
  ros::Publisher pub_clock = pnh.advertise<rosgraph_msgs::Clock>("/clock", 10);

  ros::param::set("/use_sim_time", true);

  rig::RadarManager radar_mgr(pnh);

  // Read the bag name from parameter server
  std::string bag_name;
  if (!pnh.getParam("bag_name", bag_name)) {
    ROS_ERROR("Bag name not provided");
    return 1;
  }

  std::cout << "bag_name: " << bag_name << '\n';

  // Check if the bag_name is a path ending in *
  std::vector<std::string> bag_paths;
  if (bag_name.back() == '*') {
    // Remove the * from the end of the bag_name
    bag_name.pop_back();
    std::string bag_dir = std::filesystem::path(bag_name).parent_path();
    std::string bag_pattern = std::filesystem::path(bag_name).filename().string();
    std::cout << "bag_dir: " << bag_dir << '\n';
    std::cout << "bag_pattern: " << bag_pattern << '\n';
    for (const auto & entry : std::filesystem::directory_iterator(bag_dir)) {
      if (
        entry.is_regular_file() &&
        entry.path().filename().string().find(bag_pattern) != std::string::npos) {
        bag_paths.push_back(entry.path().string());
      }
    }
  } else {
    bag_paths.push_back(bag_name);
  }

  float s_offset;
  if (!pnh.getParam("s", s_offset)) {
    s_offset = 0.0;
  }
  std::cout << "s_offset: " << s_offset << '\n';
  float duration;
  if (!pnh.getParam("duration", duration)) {
    duration = -1;
  }
  std::cout << "duration (-1 means no limit): " << duration << '\n';

  // Print out the bag_paths
  std::cout << "Opening these bags:" << '\n';
  for (const auto & path : bag_paths) {
    std::cout << path << '\n';
  }

  std::string imu_topic = pnh.resolveName("/imu");
  std::string radar_topic = pnh.resolveName("/cloud");
  std::string baro_topic = pnh.resolveName("/baro");
  std::vector<std::string> topics = {imu_topic, radar_topic, baro_topic};

  bool first = true;
  ros::Time start_time;
  for (const auto & path : bag_paths) {
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);

    // Iterate over the messages in the bag
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance & m : view) {
      if (!ros::ok()) {
        break;
      }

      ros::Time msg_time = m.getTime();
      if (first) {
        start_time = msg_time;
        first = false;
      }

      if (msg_time - start_time < ros::Duration(s_offset)) {
        continue;
      }
      if ((duration > 0) && (msg_time - start_time > ros::Duration(duration))) {
        break;
      }

      // Publish the time on the /clock topic
      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = msg_time;
      pub_clock.publish(clock_msg);

      if (m.getTopic() == radar_topic) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr) {
          radar_mgr.cloudCallback(msg);
        }
      } else if (m.getTopic() == imu_topic) {
        sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
        if (msg != nullptr) {
          radar_mgr.imuCallback(msg);
        }
      } else if (m.getTopic() == baro_topic) {
        sensor_msgs::FluidPressure::ConstPtr msg = m.instantiate<sensor_msgs::FluidPressure>();
        if (msg != nullptr) {
          radar_mgr.baroCallback(msg);
        }
      }
    }

    bag.close();
    std::cout << "Finished processing bag: " << path << '\n';
  }

  return 0;
}
