cmake_minimum_required(VERSION 3.8)
project(imu_mock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})

add_library(imu_mock SHARED src/imu_mock.cpp)
ament_target_dependencies(imu_mock rclcpp sensor_msgs)
target_link_libraries(imu_mock ${GAZEBO_LIBRARIES})

install(
  TARGETS imu_mock
  LIBRARY DESTINATION lib
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_imu_plugin test/test_imu_plugin.cpp)
  target_link_libraries(test_imu_plugin imu_mock ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_INCLUDE_DIRS})

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
