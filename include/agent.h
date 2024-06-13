#ifndef APF_AGENT_H
#define APF_AGENT_H

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "param.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace apf {
typedef Eigen::Vector3d Vector3d;
typedef std::vector<Eigen::Vector3d> Vector3ds;

class State {
public:
  Vector3d position;
  Vector3d velocity;
};

class ApfAgent : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  Param param;
  size_t agent_id = 0;
  State state;
  Vector3d start, goal;

  size_t number_of_agents = 0;
  Vector3ds positions;
  bool position_updated = false;

  void timer_callback();

  void listen_tf();

  void update_state();

  void broadcast_tf();

  Vector3d apf_controller();

public:
  ApfAgent();
};

} // namespace apf

#endif // APF_AGENT_H
