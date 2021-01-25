cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  pcl_msgs
  tf2
  tf2_ros
)

find_package(PCL 1.8 REQUIRED)

add_message_files(
  FILES
  SegmentedPointCloud.msg
)

add_service_files(
  FILES
  PointCloudIO.srv
  CurrentSegment.srv
  SegmentScene.srv
  ChangeSegmentationMethod.srv
  ChangeSegmentationMethod.srv
  EucledianClusterParams.srv
  PlaneSegmentationParams.srv
  RegionGrowingSegParams.srv
  SuperClusteringSegParams.srv
  BaseSegmentationParams.srv
)

generate_messages(DEPENDENCIES sensor_msgs pcl_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dump_scene src/dump_scene.cpp)
target_link_libraries(dump_scene ${catkin_LIBRARIES})
add_dependencies(dump_scene ${PROJECT_NAME}_gencpp)

#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


catkin_install_python(PROGRAMS
  scripts/segment_client.py
  scripts/trigger_segmentation.py
  scripts/segment_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)