#include "agent.h"

// #pragma GCC push_options
// #pragma GCC optimize("O0")

using namespace std::chrono_literals;
namespace apf {
ApfAgent::ApfAgent() : Node("agent") {
  // Agent id
  this->declare_parameter("agent_id", 0);
  agent_id = this->get_parameter("agent_id").as_int();

  // Mission
  std::string package_directory =
      ament_index_cpp::get_package_share_directory("apf");
  YAML::Node mission = YAML::LoadFile(package_directory + "/" + param.mission_file_name);
  auto agents = mission["agents"];
  start = Vector3d(agents[agent_id]["start"][0].as<double>(),
                   agents[agent_id]["start"][1].as<double>(),
                   agents[agent_id]["start"][2].as<double>());
  goal = Vector3d(agents[agent_id]["goal"][0].as<double>(),
                  agents[agent_id]["goal"][1].as<double>(),
                  agents[agent_id]["goal"][2].as<double>());
  number_of_agents = agents.size();
  positions.resize(number_of_agents);

  // State
  state.position = start;
  state.velocity = Vector3d(0, 0, 0);

  // TF2_ROS
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ROS timer
  int timer_period_ms = static_cast<int>(param.dt * 1000);
  timer = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms),
                                  std::bind(&ApfAgent::timer_callback, this));

  // Initialization finished
  std::cout << "[ApfAgent] Agent" << agent_id << " is ready." << std::endl;
}

void ApfAgent::timer_callback() {
  listen_tf();
  update_state();
  broadcast_tf();
}

void ApfAgent::listen_tf() {
  position_updated = true;
  for (size_t id = 0; id < number_of_agents; id++) {
    if (id == agent_id) {
      continue;
    }

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer->lookupTransform("world",
                                     "agent" + std::to_string(id),
                                     tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      position_updated = false;
      return;
    }

    Vector3d position = Vector3d(t.transform.translation.x,
                                 t.transform.translation.y,
                                 t.transform.translation.z);
    positions[id] = position;
  }

  // Collision check
  double min_dist = param.infinity;
  for(size_t id = 0; id < number_of_agents; id++) {
    double dist = (positions[id] - state.position).norm();
    if(id != agent_id and dist < min_dist) {
      min_dist = dist;
    }
  }
  if(min_dist < param.safety_margin){
    std::cout<< "Collision! Minimum distance between agents: " + std::to_string(min_dist) << std::endl;
  }
}

void ApfAgent::update_state() {
  if(not position_updated) {
    return;
  }

  Vector3d u = apf_controller();

  state.position += state.velocity * param.dt + 0.5 * u * param.dt * param.dt;
  state.velocity += u * param.dt;
}

void ApfAgent::broadcast_tf() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "agent" + std::to_string(agent_id);
  t.transform.translation.x = state.position.x();
  t.transform.translation.y = state.position.y();
  t.transform.translation.z = state.position.z();
  t.transform.rotation.w = 1;
  t.transform.rotation.x = 0;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  tf_broadcaster->sendTransform(t);
}

Vector3d ApfAgent::apf_controller() {
  // Attraction force
  Vector3d u_goal = param.k_goal * (goal - state.position);

  // Repulsion force
  Vector3d u_obs(0, 0, 0);
  for(size_t id = 0; id < number_of_agents; id++) {
    if(id == agent_id) {
      continue;
    }

    double dist = (positions[id] - state.position).norm();
    double obs_threshold = param.obs_threshold_ratio * param.safety_margin;
    if(dist < obs_threshold) {
      u_obs += param.k_obs * (1 / dist -  1 / obs_threshold) * (1 / (dist * dist)) *
               (state.position - positions[id]) / dist;
    }
  }

  // Damping force
  Vector3d u_damp = -param.k_damp * state.velocity;

  // Net force
  Vector3d u = u_goal + u_obs + u_damp;

  // Clamping
  for(int i = 0; i < 3; i++) {
    if(u(i) > param.max_acc) {
      u(i) = param.max_acc;
    } else if(u(i) < -param.max_acc) {
      u(i) = -param.max_acc;
    }
  }

  return u;
}
} // namespace apf

// #pragma GCC pop_options