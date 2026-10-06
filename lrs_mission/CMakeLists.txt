cmake_minimum_required(VERSION 3.8)
project(lrs_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Map representation, independent of ROS so it can be tested and reused.
add_library(voxel_map src/voxel_map.cpp)
target_include_directories(voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(voxel_map SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_map ${PCL_LIBRARIES})

add_executable(voxel_map_node src/voxel_map_node.cpp)
target_link_libraries(voxel_map_node voxel_map)
ament_target_dependencies(voxel_map_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS voxel_map voxel_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# The map lives at the repository root; ship it with the package.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../maps/ DESTINATION share/${PROJECT_NAME}/maps)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map voxel_map)
  target_compile_definitions(test_voxel_map PRIVATE
    MAP_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../maps/FEI_LRS_PCD/map.pcd")
endif()

ament_package()
