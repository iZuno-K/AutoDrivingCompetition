cmake_minimum_required(VERSION 2.8.3)
project(mypoints_downsampler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  map_file
  robot_state_publisher
  rosbridge_server
  message_generation
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  message_generation
  autoware_config_msgs
)

add_message_files(
  FILES
  PointsDownsamplerInfo.msg
  ConfigVoxelGridDistanceFilter.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  message_runtime
  autoware_config_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(voxel_grid_filter nodes/voxel_grid_filter/voxel_grid_filter.cpp)
add_dependencies(voxel_grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

install(TARGETS voxel_grid_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)


