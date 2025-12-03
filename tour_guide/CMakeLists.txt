cmake_minimum_required(VERSION 3.0.2)
project(tour_guide)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  sound_play
  std_msgs
  tf
  kobuki_msgs
  image_transport      # <-- added
  tf2_ros              # <-- added
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## (all the message/service/action stuff left commented, as before)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tour_guide
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    move_base_msgs
    nav_msgs
    rospy
    sensor_msgs
    sound_play
    std_msgs
    tf
    kobuki_msgs
    image_transport
    tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## (no C++ targets; all left commented)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/goal_runner.py
  scripts/help_watchdog.py
  scripts/checkerboard_ir.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## (everything else left as-is / commented)

#############
## Testing ##
#############
## (tests left commented)
