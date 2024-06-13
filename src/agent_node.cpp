#include "agent.h"

using namespace apf;
int main(int argc, const char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfAgent>());
  rclcpp::shutdown();
  return 0;
}