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
  //TODO: initialize tf
  tf_buffer = TODO;
  tf_listener = TODO;
  tf_broadcaster = TODO;

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
      //TODO: Get the position of the other agents
      t = TODO;
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

  // Compute the control input
  Vector3d u = apf_controller();

  //TODO: Update the state of the double integrator model using the control input u
  state.position = TODO;
  state.velocity = TODO;
}

void ApfAgent::broadcast_tf() {
  //TODO: Broadcast the agent's current position using 'state'
}

Vector3d ApfAgent::apf_controller() {
  // Attraction force
  Vector3d u_goal = TODO;

  // Repulsion force
  Vector3d u_obs = TODO;

  // Damping force
  Vector3d u_damp = -param.k_damp * state.velocity;

  // Net force
  Vector3d u = u_goal + u_obs + u_damp;

  // Clamping for maximum acceleration constraint
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