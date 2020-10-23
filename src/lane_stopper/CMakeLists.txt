cmake_minimum_required(VERSION 2.8.3)
project(lane_stopper)

find_package(autoware_msgs REQUIRED)
find_package(autoware_build_flags REQUIRED)

find_package(autoware_config_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        std_msgs
        sensor_msgs
        autoware_msgs
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
        autoware_msgs
        autoware_config_msgs
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
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(lane_stopper
        nodes/lane_stopper.cpp
        )

target_link_libraries(lane_stopper
        ${catkin_LIBRARIES}
        )
        

add_dependencies(lane_stopper ${catkin_EXPORTED_TARGETS})

install(TARGETS
        lane_stopper
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
