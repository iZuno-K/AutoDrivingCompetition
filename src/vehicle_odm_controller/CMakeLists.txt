cmake_minimum_required(VERSION 3.0.2)
SET(pkg_name vehicle_odm_controller)
project(${pkg_name})

find_package(autoware_msgs REQUIRED)
find_package(autoware_build_flags REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(autoware_config_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  angles
  autoware_config_msgs
  autoware_config_msgs
  geometry_msgs
  autoware_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  std_msgs
)

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        autoware_msgs
        autoware_config_msgs
        nav_msgs
        tf
        tf2
        tf2_ros
        tf2_eigen
        autoware_health_checker
        )

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        include/${pkg_name}
        Eigen3::Eigen
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${pkg_name} 
        # ${SRC_FILES}
        src/vehicle_state.cpp
        src/vehicle_state_node.cpp
        )

target_link_libraries(${pkg_name}
        ${catkin_LIBRARIES}
        )

add_dependencies(${pkg_name} ${catkin_EXPORTED_TARGETS})

install(TARGETS
        ${pkg_name}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

# Neccesary for launch file

# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#         PATTERN ".svn" EXCLUDE)