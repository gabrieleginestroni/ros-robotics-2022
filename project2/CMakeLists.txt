cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rospy
  message_generation
)

add_service_files(
        FILES
        DrawPath.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf
  src/odom_tf.cpp
)
add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf
  ${catkin_LIBRARIES}
)

