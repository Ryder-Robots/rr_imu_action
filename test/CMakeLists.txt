find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_add_gmock(${PROJECT_NAME}_rr_imu_action_node_test
    rr_imu_action_node_test.cpp
)

target_link_libraries(${PROJECT_NAME}_rr_imu_action_node_test  rr_imu_action_node)
