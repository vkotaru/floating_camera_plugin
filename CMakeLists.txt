cmake_minimum_required(VERSION 2.8.3)
project(floating_camera_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_plugins
        gazebo_ros
        nav_msgs
        geometry_msgs
        )
find_package(gazebo REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros nav_msgs geometry_msgs
        DEPENDS EIGEN3 GAZEBO
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(SRCS
        src/floating_camera_plugin.cc)

add_library(floating_camera_plugin SHARED ${SRCS})
target_link_libraries(floating_camera_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(floating_camera_plugin ${catkin_EXPORTED_TARGETS})
