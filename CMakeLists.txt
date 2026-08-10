cmake_minimum_required(VERSION 3.10)
project(k4a_ros VERSION 2.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Azure Kinect SDK installs a CMake config package exporting k4a::k4a.
find_package(k4a 1.4.2 CONFIG REQUIRED)

add_library(k4a_driver_component SHARED
  src/k4a_driver_node.cpp
)
target_include_directories(k4a_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(k4a_driver_component PRIVATE K4A_ROS_BUILDING_DLL)
ament_target_dependencies(k4a_driver_component
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(k4a_driver_component k4a::k4a)

rclcpp_components_register_node(k4a_driver_component
  PLUGIN "k4a_ros::K4ADriverNode"
  EXECUTABLE k4a_driver_component_node
  EXECUTOR SingleThreadedExecutor
)

add_executable(k4a_driver_node src/k4a_driver_main.cpp)
target_link_libraries(k4a_driver_node k4a_driver_component)
ament_target_dependencies(k4a_driver_node rclcpp)

install(
  TARGETS k4a_driver_component k4a_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/k4a_ros DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(
  FILES CHANGELOG.md LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/validate_streams.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(k4a_ros_cpp_files
    include/k4a_ros/k4a_driver_node.hpp
    src/k4a_driver_main.cpp
    src/k4a_driver_node.cpp
  )
  ament_cppcheck(${k4a_ros_cpp_files})
  ament_cpplint(${k4a_ros_cpp_files})
  ament_uncrustify(${k4a_ros_cpp_files})
  ament_flake8(launch/k4a.launch.py scripts/validate_streams.py)
  ament_pep257(launch/k4a.launch.py)
  ament_lint_cmake(CMakeLists.txt)
  ament_xmllint(package.xml)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)
ament_package()
