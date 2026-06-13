cmake_minimum_required(VERSION 3.16)
project(imu_attitude_estimation_gtsam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTSAM REQUIRED)

add_executable(gtsam_fgo_node src/gtsam_fgo_node.cpp)
target_include_directories(gtsam_fgo_node PRIVATE ${GTSAM_INCLUDE_DIR})
ament_target_dependencies(
  gtsam_fgo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)
target_link_libraries(gtsam_fgo_node gtsam metis-gtsam)
set_target_properties(gtsam_fgo_node PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
)

install(TARGETS gtsam_fgo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
