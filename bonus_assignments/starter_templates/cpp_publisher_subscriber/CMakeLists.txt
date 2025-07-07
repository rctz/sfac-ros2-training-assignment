cmake_minimum_required(VERSION 3.8)
project(cpp_sensor_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# TODO: Add custom message package dependency
# find_package(robot_sensor_publisher REQUIRED)

# TODO: Create executable for publisher
# add_executable(sensor_publisher src/sensor_publisher.cpp)
# ament_target_dependencies(sensor_publisher
#   rclcpp
#   std_msgs
#   robot_sensor_publisher
# )

# TODO: Create executable for subscriber  
# add_executable(data_processor src/data_processor.cpp)
# ament_target_dependencies(data_processor
#   rclcpp
#   std_msgs
#   robot_sensor_publisher
# )

# TODO: Install executables
# install(TARGETS
#   sensor_publisher
#   data_processor
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 