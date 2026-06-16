cmake_minimum_required(VERSION 3.8)
project(hdl_localization)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_definitions(-std=c++17)
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
	# -mavx causes a lot of errors!!
  add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL REQUIRED)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(hdl_global_localization REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanMatchingStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(hdl_localization_component
  src/hdl_localization/pose_estimator.cpp
  apps/hdl_localization.cpp
)
target_include_directories(hdl_localization_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hdl_localization_component
  tf2_ros 
  tf2_eigen 
  pcl_ros 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  ndt_omp 
  fast_gicp 
  hdl_global_localization
  std_srvs
  nav_msgs
  rclcpp_components
  std_msgs
)
target_link_libraries(hdl_localization_component
  ${PCL_LIBRARIES}  "${cpp_typesupport_target}"
)

rclcpp_components_register_node(
    hdl_localization_component
    PLUGIN "hdl_localization::HdlLocalization"
    EXECUTABLE hdl_localization_node
)


add_library(globalmap_server_component
  apps/globalmap_server.cpp
)
target_include_directories(globalmap_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(globalmap_server_component
  tf2_ros 
  tf2_eigen
  pcl_ros 
  rclcpp
  sensor_msgs
  geometry_msgs
  ndt_omp 
  fast_gicp 
  hdl_global_localization
  std_srvs
  nav_msgs
  rclcpp_components
  std_msgs
)
target_link_libraries(globalmap_server_component
  ${PCL_LIBRARIES}  "${cpp_typesupport_target}"
)

rclcpp_components_register_node(
    globalmap_server_component
    PLUGIN "hdl_localization::GlobalmapServer"
    EXECUTABLE globalmap_server
)


install(DIRECTORY launch rviz param
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_hdl_localization_component)
install(TARGETS hdl_localization_component
        EXPORT export_hdl_localization_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_globalmap_server_component)
install(TARGETS globalmap_server_component
        EXPORT export_globalmap_server_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/plot_status.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()