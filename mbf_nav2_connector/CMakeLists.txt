cmake_minimum_required(VERSION 3.8)
project(mbf_nav2_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mbf_simple_core REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  pluginlib
  mbf_simple_core
  mbf_utility
  mbf_msgs
  nav2_msgs
  nav2_core
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Nav2 Planner Plugin
add_library(nav2_planner_plugin SHARED
  src/nav2_planner.cpp
)
target_include_directories(nav2_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_planner_plugin ${dependencies})

# Nav2 Controller Plugin
add_library(nav2_controller_plugin SHARED
  src/nav2_controller.cpp
)
target_include_directories(nav2_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav2_controller_plugin ${dependencies})

# Install libraries
install(TARGETS nav2_planner_plugin nav2_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES
  mbf_nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(mbf_simple_core mbf_nav2_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_planner_plugin nav2_controller_plugin)
ament_export_dependencies(${dependencies})
ament_package()
