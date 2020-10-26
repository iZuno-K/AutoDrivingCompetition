cmake_minimum_required(VERSION 2.8.3)
project(aichallenge_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  map_file
  robot_state_publisher
  rosbridge_server
  message_generation
)

add_message_files(
  FILES
  ScoreData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch data scenario)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

set(PYTHON_SCRIPTS 
  scripts/initial_pose_publisher.py
  scripts/initial_pose_publisher_intersection1.py
  scripts/initial_pose_publisher_intersection2.py
  scripts/yaml_to_camera_info_publisher.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)
