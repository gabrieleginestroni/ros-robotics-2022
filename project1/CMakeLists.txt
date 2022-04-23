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
        message_generation
        )

## Services
add_service_files(
        FILES
        reset_odom_to_pose.srv
)

## Parameters config file
generate_dynamic_reconfigure_options(
        cfg/parameters.cfg
)

## Message dependencies
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Executables definitions
add_executable(odom_pub src/odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})

add_executable(vel_pub src/vel_pub.cpp)
target_link_libraries(vel_pub ${catkin_LIBRARIES})

add_executable(errorComp src/errorComp.cpp)
target_link_libraries(errorComp ${catkin_LIBRARIES})

add_dependencies(odom_pub ${catkin_EXPORTED_TARGETS})
