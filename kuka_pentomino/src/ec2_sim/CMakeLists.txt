cmake_minimum_required(VERSION 2.8.3)
project(ec2_sim)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  angles 
  geometry_msgs 
  sensor_msgs 
  tf 
  tf2_ros 
  dynamic_reconfigure 
  image_transport 
  cv_bridge
  camera_info_manager
  std_msgs
  trac_ik_lib
  eigen_conversions
  euroc_ros_msgs
)

find_package(gazebo REQUIRED)
set(gazebo_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
set(gazebo_LIBRARIES ${GAZEBO_LIBRARIES})

pkg_check_modules(SDF sdformat REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/GazeboRosCamera.cfg
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
  gazebo_ros_camera_utils 
  gazebo_ros_depth_camera 
  
  CATKIN_DEPENDS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  angles 
  geometry_msgs 
  sensor_msgs 
  tf 
  tf2_ros 
  image_transport 
  cv_bridge
  camera_info_manager
  std_msgs
  DEPENDS 
    gazebo 
    SDF
  )
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

#add_library(ec2_gazebo_ros_camera_utils src/gazebo_ros_camera_utils.cpp)
#add_dependencies(ec2_gazebo_ros_camera_utils ${PROJECT_NAME}_gencfg)
#target_link_libraries(ec2_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_library(ec2_gazebo_ros_depth_camera src/gazebo_ros_depth_camera.cpp)
#add_dependencies(ec2_gazebo_ros_depth_camera ${PROJECT_NAME}_gencfg)
#target_link_libraries(ec2_gazebo_ros_depth_camera ec2_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})

add_executable(inverse_kinematic_service src/InverseKinematicService.cpp)
target_link_libraries(inverse_kinematic_service ${catkin_LIBRARIES})
add_dependencies(inverse_kinematic_service trac_ik_lib)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ec2_gazebo_ros_depth_camera ec2_gazebo_ros_camera_utils
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


