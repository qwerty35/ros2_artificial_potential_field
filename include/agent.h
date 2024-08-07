#ifndef APF_AGENT_H
#define APF_AGENT_H

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <utility>

namespace apf {
typedef Eigen::Vector3d Vector3d;
typedef std::vector<Eigen::Vector3d> Vector3ds;

class State {
public:
  Vector3d position;
  Vector3d velocity;
};

class Obstacle {
public:
  Vector3d position;
  double radius;
};

class ApfAgent : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_tf, timer_pub;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose;

  // Simulator
  std::string frame_id = "world"; // source frame id for tf broadcast 
  double dt = 0.02;               // timer period (s)

  // Agent
  size_t agent_id = 0;  // each agent has unique agent_id
  State state;          // agent's state
  Vector3d start, goal; // agent's start and goal positions
  double radius = 0.15; // (m)
  double max_acc = 6;   // (m/s^2)

  // Environment
  size_t number_of_agents = 0;
  size_t number_of_obstacles = 0;
  Vector3ds agent_positions;       // agent's positions, including the current agent itself
  std::vector<Obstacle> obstacles; // obstacles list

  void timer_tf_callback();
  
  void timer_pub_callback();
  
  void listen_tf();

  void update_state();

  void broadcast_tf();

  Vector3d apf_controller();

  void publish_marker_pose();

public:
  ApfAgent();
};

} // namespace apf

#endif // APF_AGENT_H
