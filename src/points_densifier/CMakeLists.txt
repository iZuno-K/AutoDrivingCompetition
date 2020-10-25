cmake_minimum_required(VERSION 2.8.3)
project(points_densifier)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_config_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        autoware_config_msgs
        tf
        tf2
        tf2_ros
        tf2_eigen
        autoware_health_checker
        )

catkin_package(CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        autoware_config_msgs
        tf
        tf2
        tf2_ros
        tf2_eigen
        autoware_health_checker
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# Points Concat filter
add_executable(points_densifier
        src/points_densifier.cpp
        )

target_link_libraries(points_densifier
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

add_dependencies(points_densifier ${catkin_EXPORTED_TARGETS})

install(TARGETS
        points_densifier
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
