#ifndef APF_PARAM_H
#define APF_PARAM_H

#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <utility>

namespace apf {

class Param {
public:
  // Config files
  std::string mission_file_name = "mission/mission_circle_30.yaml";
  double dt = 0.02;

  // Agent
  double safety_margin = 0.15; // (m)
  double max_acc = 10;   // (m/s^2)

  // APF
  double k_goal = 0.7;
  double k_obs = 20;
  double k_damp = 3;
  double obs_threshold_ratio = 5;

  // Constant
  double infinity = 100000;

  Param() = default;
};

} // namespace apf

#endif // APF_PARAM_H
