cmake_minimum_required(VERSION 2.8.3)

# Project name and the type of project
PROJECT(k4a-ros)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(
    catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        image_transport
        cv_bridge
)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(PCL REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(include)



# add other necessary include directories
include_directories(
	src/
	${catkin_INCLUDE_DIRS}
	)

# add_service_files(
#   FILES
#   PointCloudAsService.srv
# )

## Generate added messages and services
# generate_messages(
#       DEPENDENCIES
#       std_msgs
#       sensor_msgs
# )

# dynamic reconfigure cfg file
# generate_dynamic_reconfigure_options(cfg/gocator_3100_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS
        roscpp
        rospy
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)

#Build executable
add_executable(k4a_ros_node src/k4a_ros_node.cpp src/libs/StaticImageProperties.cpp)
add_dependencies(k4a_ros_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(k4a_ros_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} k4a)


#Build examples
add_subdirectory(src/examples)
