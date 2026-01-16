cmake_minimum_required(VERSION 3.8)
project(swarm_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/DroneIntent.msg"
  "msg/FormationRole.msg"
  "msg/SwarmStatus.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/VIOState.msg"
  "srv/SetFormation.srv"
  "srv/EmergencyCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Find Python for correct site-packages path
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install Python modules to correct site-packages (matching rosidl output)
install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  scripts/swarm_bridge
  scripts/drone_node
  scripts/mission_coordinator
  scripts/perception_node
  scripts/vio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
