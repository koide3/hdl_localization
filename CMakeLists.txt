cmake_minimum_required(VERSION 2.8.3)
project(hdl_localization)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_definitions(-std=c++11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
	# -mavx causes a lot of errors!!
  if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
    add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  else()
    add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  endif()
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  ndt_omp
  fast_gicp
  hdl_global_localization
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

########################
## message generation ##
########################
add_message_files(FILES
  ScanMatchingStatus.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# nodelets
add_library(hdl_localization_nodelet
  src/hdl_localization/pose_estimator.cpp
  apps/hdl_localization_nodelet.cpp
)
target_link_libraries(hdl_localization_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_localization_nodelet ${PROJECT_NAME}_gencpp)


add_library(globalmap_server_nodelet apps/globalmap_server_nodelet.cpp)
target_link_libraries(globalmap_server_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

