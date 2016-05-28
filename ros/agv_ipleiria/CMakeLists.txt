cmake_minimum_required(VERSION 2.8.3)
project(agv_ipleiria)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  geometry_msgs
  std_msgs
  genmsg
  nav_msgs 
  message_generation 
)  

# Declare the project as a catkin package
#catkin_package()

# Check for launch files
roslaunch_add_file_check(launch)

# Add header folders
include_directories(include ${catkin_INCLUDE_DIRS})



 
 
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
 

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   AgvCommand.msg
   AgvStatus.msg 
   RangeArray.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Agv_sensors.srv
   Agv_main.srv
#   Service1.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
     geometry_msgs
     nav_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include 
  LIBRARIES agv_ipleiria geometry_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a cpp library
# add_library(agv_ipleiria
#   src/${PROJECT_NAME}/agv_ipleiria.cpp
# )

## Declare a cpp executable
# add_executable(agv_ipleiria_node src/agv_ipleiria_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(agv_ipleiria_node agv_ipleiria_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(agv_ipleiria_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS agv_ipleiria agv_ipleiria_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agv_ipleiria.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(kb_control src/kb_control.cpp)
target_link_libraries(kb_control ${catkin_LIBRARIES})
add_dependencies(kb_control agv_ipleiria_generate_messages_cpp)


add_executable(agvipleiria_demo src/agvipleiria_demo.cpp)
target_link_libraries(agvipleiria_demo ${catkin_LIBRARIES})
add_dependencies(agvipleiria_demo agv_ipleiria_generate_messages_cpp)


add_executable(agvmain_menu src/agvmain_menu.cpp)
target_link_libraries(agvmain_menu ${catkin_LIBRARIES})
add_dependencies(agvmain_menu agv_ipleiria_generate_messages_cpp)

add_executable(sensors_menu src/sensors_menu.cpp)
target_link_libraries(sensors_menu ${catkin_LIBRARIES})
add_dependencies(sensors_menu agv_ipleiria_generate_messages_cpp)
