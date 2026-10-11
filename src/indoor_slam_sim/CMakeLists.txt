cmake_minimum_required(VERSION 3.8)
project(indoor_slam_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
add_library(indoor_imu_plugin SHARED src/indoor_sim_imu_plugin.cpp)
target_include_directories(indoor_imu_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(indoor_imu_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(indoor_imu_plugin gazebo_ros rclcpp sensor_msgs)
target_link_libraries(indoor_imu_plugin ${GAZEBO_LIBRARIES})
set_target_properties(indoor_imu_plugin PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(indoor_mid360_gpu_plugin SHARED src/indoor_mid360_gpu_plugin.cpp)
target_include_directories(indoor_mid360_gpu_plugin PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_directories(indoor_mid360_gpu_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(indoor_mid360_gpu_plugin gazebo_ros rclcpp sensor_msgs)
target_link_libraries(indoor_mid360_gpu_plugin ${GAZEBO_LIBRARIES})
set_target_properties(indoor_mid360_gpu_plugin PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS indoor_imu_plugin indoor_mid360_gpu_plugin LIBRARY DESTINATION lib)
install(DIRECTORY launch worlds urdf models config rviz
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_limiter.py scripts/check_goal_geometry.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
