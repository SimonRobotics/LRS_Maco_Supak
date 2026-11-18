cmake_minimum_required(VERSION 3.8)
project(lrs_localization)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(localization_node src/localization_node.cpp)
target_include_directories(localization_node PRIVATE include)
ament_target_dependencies(localization_node rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_channel test/test_sensor_channel.cpp)
  target_include_directories(test_sensor_channel PRIVATE include)
endif()

ament_package()
