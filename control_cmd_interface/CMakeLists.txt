cmake_minimum_required(VERSION 2.8.3)
project(control_cmd_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp std_msgs message_runtime message_generation )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")




################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include ${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


set(H_FILES  	${INCLUDE_DIR}/control_cmd_interface/control_cmd_interface.h
                ${INCLUDE_DIR}/ros_param_parser/ros_param_parser.h
)

set(SRC_FILES 	${SRC_DIR}/control_cmd_interface/control_cmd_interface.cpp
)


add_library(${PROJECT_NAME} SHARED ${H_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_node ${SRC_DIR}/control_cmd_interface_node.cpp )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

