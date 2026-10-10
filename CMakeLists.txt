cmake_minimum_required(VERSION 3.20)
project(easynav_nav2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(easynav_system REQUIRED)
find_package(easynav_interfaces REQUIRED)

set(dependencies
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  easynav_system::easynav_system
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${easynav_interfaces_TARGETS}
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_nav2_bridge/Nav2Bridge.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${dependencies}
)

add_executable(nav2_bridge_main src/nav2_bridge_main.cpp)
target_link_libraries(nav2_bridge_main PUBLIC
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  nav2_bridge_main
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  easynav_system
  easynav_interfaces
)
ament_package()
