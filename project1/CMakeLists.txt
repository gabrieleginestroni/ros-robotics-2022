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
        tf2
        tf2_ros
        message_generation
        )

add_message_files(
        FILES
        RpmStamped.msg
)

## Services
add_service_files(
        FILES
        reset_odom_to_pose.srv
)

## Message dependencies
generate_messages(DEPENDENCIES std_msgs)

## Parameters config file
generate_dynamic_reconfigure_options(
        cfg/parameters.cfg
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Executables definitions
add_executable(odom_pub src/odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})

add_executable(vel_pub src/vel_pub.cpp)
target_link_libraries(vel_pub ${catkin_LIBRARIES})

add_executable(inverter src/inverter.cpp)
target_link_libraries(inverter ${catkin_LIBRARIES})

add_dependencies(inverter ${catkin_EXPORTED_TARGETS})

add_dependencies(odom_pub ${catkin_EXPORTED_TARGETS})
