cmake_minimum_required(VERSION 2.8.3)
project(MapManager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_core
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        geometry_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(serial)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES trajectory_planning
        CATKIN_DEPENDS nav_core roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_library(MapManager
        include/MapManager/MapManager.h
        src/MapManager/MapManager.cpp)

add_executable(main App/main.cpp
        )
target_link_libraries(main
        ${catkin_LIBRARIES}
        MapManager
)


