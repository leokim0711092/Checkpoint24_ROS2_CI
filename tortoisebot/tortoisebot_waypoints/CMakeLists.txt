cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tortoisebot_waypoints REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/test_client.cpp
      )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test waypoint_gtest)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action std_msgs geometry_msgs rclcpp_action tortoisebot_waypoints nav_msgs )
  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()
endif()

## ADD LIBRARY
include_directories(include)
add_library(waypoint_gtest src/tortoisebot_action_server.cpp )
ament_target_dependencies(waypoint_gtest
  rclcpp rclcpp_action std_msgs geometry_msgs rclcpp_action tortoisebot_waypoints nav_msgs)


add_executable(Waypoint_node src/tortoisebot_action_server.cpp)
ament_target_dependencies(Waypoint_node rclcpp rclcpp_action std_msgs geometry_msgs rclcpp_action tortoisebot_waypoints nav_msgs )



install(TARGETS
	Waypoint_node
    DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
