cmake_minimum_required(VERSION 3.8)
project(sura_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/auv_controllers/body_velocity_controller.cpp
  src/auv_controllers/depth_hold_controller.cpp
  src/auv_controllers/mpc_4dof_controller.cpp
  src/auv_controllers/position_hold_controller.cpp
  src/auv_controllers/state_observer_controller.cpp
  src/auv_controllers/state_observer_model.cpp
  src/auv_controllers/stabilize_controller.cpp
  src/auv_controllers/simple_observer_controller.cpp
  src/common_controllers/body_force_controller.cpp
  src/usv_controllers/position_hold_controller.cpp
  src/usv_controllers/body_velocity_controller.cpp
  src/uvms_controllers/alpha_cartesian_velocity_controller.cpp
)

add_executable(controller_arbitrator
  src/controller_arbitrator.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_features(controller_arbitrator PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(controller_arbitrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  osqp::osqp
  tinyxml2::tinyxml2
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  sura_msgs
  urdf
  kdl_parser
  lifecycle_msgs
  orocos_kdl
  tf2
  visualization_msgs
)

ament_target_dependencies(controller_arbitrator
  controller_manager_msgs
  geometry_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  sura_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  sura_controllers_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS controller_arbitrator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES sura_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  osqp_vendor
  sura_msgs
  urdf
  kdl_parser
  lifecycle_msgs
  orocos_kdl
  tf2
  yaml-cpp
  visualization_msgs
)

ament_package()
