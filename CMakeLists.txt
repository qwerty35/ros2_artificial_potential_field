cmake_minimum_required(VERSION 3.8)
project(apf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

#YAML
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

#EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(ROS2_PACKAGES
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  visualization_msgs
  ament_index_cpp)

#MULTI_AMR_PLANNER
set(AGENT_SRC
        src/agent.cpp
  src/agent_node.cpp)

#########
# Build #
#########
add_executable(agent_node ${AGENT_SRC})
target_link_libraries(agent_node
  ${YAML_CPP_LIBRARIES})
ament_target_dependencies(agent_node ${ROS2_PACKAGES})
install(TARGETS agent_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config mission mesh
  DESTINATION share/${PROJECT_NAME})

ament_package()
