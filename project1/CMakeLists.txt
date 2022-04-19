cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        dynamic_reconfigure
        tf
        )

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_pub src/odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})

add_executable(vel_pub src/vel_pub.cpp)
target_link_libraries(vel_pub ${catkin_LIBRARIES})
