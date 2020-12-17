cmake_minimum_required(VERSION 2.8.3)
project(ec2_skills_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    euroc_ros_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
    FILES
    MovePlatform.srv
    MoveArm.srv
    MovePT.srv
    PickObject.srv
    PlaceObject.srv
    PushObject.srv
    FlipP5.srv
    PickBlanks.srv
    PlaceBlanks.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    euroc_ros_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
