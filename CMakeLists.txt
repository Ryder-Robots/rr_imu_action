# Copyright (c) 2025 Ryder Robots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.20)
project(rr_imu_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(rr_imu_action_node src/rr_imu_action_node.cpp)
add_library(rr_imu_action::rr_imu_action_node ALIAS rr_imu_action_node)
target_compile_features(rr_imu_action_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rr_imu_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  rr_imu_action_node PUBLIC
  ${rr_common_base_TARGETS}
  ${rr_interfaces_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${rclcpp_components_TARGETS}
  ${pluginlib_TARGETS}
  ${rclcpp_action_TARGETS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_imu_action_node PRIVATE "RR_IMU_ACTION_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rr_imu_action_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  rr_imu_action_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

rclcpp_components_register_node(rr_imu_action_node
  PLUGIN "rr_imu_action::RrImuActionNode"
  EXECUTABLE "rr_imu_action"
)

ament_package()
