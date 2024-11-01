cmake_minimum_required(VERSION 3.8)
project(wander_and_avoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED) 


add_executable(wander_and_avoid src/wander_and_avoid.cpp)
add_executable(follower_robot src/follower_robot.cpp)


ament_target_dependencies(wander_and_avoid rclcpp geometry_msgs sensor_msgs nav_msgs)
ament_target_dependencies(follower_robot rclcpp tf2_ros geometry_msgs sensor_msgs)

install(TARGETS
  wander_and_avoid
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  follower_robot
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Add this line to the end of CMakeLists.txt
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
