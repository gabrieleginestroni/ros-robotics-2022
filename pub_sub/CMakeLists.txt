cmake_minimum_required(VERSION 2.8.3)
project(pub_sub)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sub_abs src/sub_abs_ref.cpp)
target_link_libraries(sub_abs ${catkin_LIBRARIES})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
