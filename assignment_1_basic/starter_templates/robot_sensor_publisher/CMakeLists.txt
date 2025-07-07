cmake_minimum_required(VERSION 3.8)
project(robot_sensor_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# TODO: Add rosidl dependencies for message generation
# find_package(rosidl_default_generators REQUIRED)

# TODO: Add message files
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SensorData.msg"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# TODO: Export dependencies for other packages to use your messages
# ament_export_dependencies(rosidl_default_runtime)

ament_package() 