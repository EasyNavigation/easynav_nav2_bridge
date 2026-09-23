find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(nav_msgs REQUIRED)

ament_add_gtest(nav2_bridge_tests nav2_bridge_tests.cpp)
target_link_libraries(nav2_bridge_tests
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  ${nav_msgs_TARGETS}
)
