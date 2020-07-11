cmake_minimum_required(VERSION 2.8.3)
project(rovitis_phidgets)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  nodelet
  pluginlib
  #phidgets_api
  message_generation
  genmsg
  message_runtime
##  rovitis_row_navigation
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
   phidget_frequency_counter.msg
   phidget_high_speed_encoder.msg
   phidget_interface_kit_888.msg
   phidget_interface_kit_004.msg
   phidget_analog_4_output.msg
   phidget_motor_control_hc.msg
   phidget_spatial.msg
   phidget_stepper_bipolar.msg
   phidget_gps.msg
 )

## Generate services in the '' folder
 add_service_files(
   FILES
   phidget_frequency_counter_reset.srv
   phidget_interface_kit_888_set.srv
   phidget_interface_kit_004_set.srv
   phidget_analog_4_output_set.srv
   phidget_spatial_set.srv
   phidget_stepper_bipolar_set.srv
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
   geometry_msgs   nav_msgs   sensor_msgs  std_msgs
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
  INCLUDE_DIRS include
  #LIBRARIES rovitis_phidgets
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf nodelet pluginlib message_generation 
  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rovitis_phidgets
#   src/${PROJECT_NAME}/rovitis_phidgets.cpp
# )

link_directories(/opt/ros/kinetic/lib)

## Declare a cpp executable
# add_executable(rovitis_phidgets_node src/rovitis_phidgets_node.cpp)
## Declare a cpp executable
#add_executable(phidgets_interface_kit_888
#    include/rovitis_phidgets/phidgets_interface_kit_888.hpp
#    src/phidgets_interface_kit_888.cpp
#)

#add_executable(phidgets_frequency_counter
#    include/rovitis_phidgets/phidgets_frequency_counter.hpp
#    src/phidgets_frequency_counter.cpp
#)

#add_executable(phidgets_high_speed_encoder
#    include/rovitis_phidgets/phidgets_high_speed_encoder.hpp
#    src/phidgets_high_speed_encoder.cpp
#)

add_executable(phidgets_interface_kit_004
    include/rovitis_phidgets/phidgets_interface_kit_004.hpp
    src/phidgets_interface_kit_004.cpp
)

#add_executable(phidgets_analog_4_output
#    include/rovitis_phidgets/phidgets_analog_4_output.hpp
#    src/phidgets_analog_4_output.cpp
#)

#add_executable(phidgets_motor_control_hc
#    include/rovitis_phidgets/phidgets_motor_control_hc.hpp
#    src/phidgets_motor_control_hc.cpp
#)

#add_executable(phidgets_spatial
#    include/rovitis_phidgets/phidgets_spatial.hpp
#    src/phidgets_spatial.cpp
#)

#add_executable(phidgets_stepper_bipolar
#    include/rovitis_phidgets/phidgets_stepper_bipolar.hpp
#    src/phidgets_stepper_bipolar.cpp
#)

#add_executable(phidgets_gps
#    include/rovitis_phidgets/phidgets_gps.hpp
#    src/phidgets_gps.cpp
#)

add_executable(phidgets_list_devices
    src/phidgets_list_devices.cpp
)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_interface_kit_888 phidgets_interface_kit_888_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_frequency_counter phidgets_frequency_counter_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_high_speed_encoder phidgets_high_speed_encoder_gencpp)

## Add cmake target dependencies of the executable/library
add_dependencies(phidgets_interface_kit_004 phidgets_interface_kit_004_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_analog_4_output phidgets_analog_4_output_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_motor_control_hc phidgets_motor_control_hc_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_spatial phidgets_spatial_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_stepper_bipolar phidgets_stepper_bipolar_gencpp)

## Add cmake target dependencies of the executable/library
#add_dependencies(phidgets_gps phidgets_gps_gencpp)

## Add cmake target dependencies of the executable/library
add_dependencies(phidgets_list_devices phidgets_list_devices_gencpp)

#target_link_libraries(phidgets_interface_kit_888 phidget21)
#target_link_libraries(phidgets_frequency_counter phidget21)
#target_link_libraries(phidgets_high_speed_encoder phidget21)
target_link_libraries(phidgets_interface_kit_004 phidget21)
#target_link_libraries(phidgets_analog_4_output phidget21)
#target_link_libraries(phidgets_motor_control_hc phidget21)

#target_link_libraries(phidgets_stepper_bipolar phidget21)
#target_link_libraries(phidgets_gps phidget21)
#target_link_libraries(phidgets_spatial phidget21)
target_link_libraries(phidgets_list_devices phidget21)

## Specify libraries to link a library or executable target against
#target_link_libraries(phidgets_interface_kit_888
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_frequency_counter
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_high_speed_encoder
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

target_link_libraries(phidgets_interface_kit_004
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#target_link_libraries(phidgets_analog_4_output
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_motor_control_hc
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_stepper_bipolar
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_gps
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(phidgets_spatial
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

target_link_libraries(phidgets_list_devices
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(phidgets_interface_kit_888 rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_frequency_counter rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_high_speed_encoder rovitis_phidgets_generate_messages_cpp)

add_dependencies(phidgets_interface_kit_004 rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_analog_4_output rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_motor_control_hc rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_stepper_bipolar rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_spatial rovitis_phidgets_generate_messages_cpp)

#add_dependencies(phidgets_gps rovitis_phidgets_generate_messages_cpp)

add_dependencies(phidgets_list_devices rovitis_phidgets_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(rovitis_phidgets_node
#   ${catkin_LIBRARIES}
#)

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
# install(TARGETS rovitis_phidgets rovitis_phidgets_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rovitis_phidgets.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
