cmake_minimum_required(VERSION 3.0.2)
project(ros_param_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs    
  ros_numpy
  cv_bridge
  message_filters
  roscpp  
)

# catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES train_control
#  CATKIN_DEPENDS ros_numpy cv_bridge message_filters roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS 
#   src/detect.py
#   src/track.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
