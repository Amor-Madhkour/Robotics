cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
    nav_msgs
    dynamic_reconfigure
    tf
    tf2
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    StampedWheelRPM.msg

)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    reset_odom.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
 cfg/parameters.cfg
)

## Declare things to be passed to dependent projects
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs

)

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vel_pub src/vel_pub.cpp)
add_dependencies(vel_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(vel_pub ${catkin_LIBRARIES})

add_executable(odom_pub src/odom_pub.cpp)
add_dependencies(odom_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_pub ${catkin_LIBRARIES})

add_executable(wheelRPM_pub src/wheelRPM_pub.cpp)
add_dependencies(wheelRPM_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelRPM_pub ${catkin_LIBRARIES})

add_executable(dynamic_reconfigure src/dynamic_reconfigure.cpp)
add_dependencies(dynamic_reconfigure ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_reconfigure ${catkin_LIBRARIES})

add_executable(transform_manager src/transform_manager.cpp)
add_dependencies(transform_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_manager ${catkin_LIBRARIES})