cmake_minimum_required(VERSION 3.8)
project(lrs_mission)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # A* and inflation are ~10x slower in Debug
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(lrs_mission_node src/lrs_mission_node.cpp)
target_include_directories(lrs_mission_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lrs_mission_node ${PCL_LIBRARIES})
ament_target_dependencies(lrs_mission_node
  rclcpp geometry_msgs mavros_msgs nav_msgs sensor_msgs std_msgs std_srvs visualization_msgs)

install(TARGETS lrs_mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
# Map and mission files live at the repository root; ship them with the package.
install(DIRECTORY ../maps ../missions DESTINATION share/${PROJECT_NAME})

ament_package()
