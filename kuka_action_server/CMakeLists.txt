cmake_minimum_required(VERSION 2.8.3)
project(kuka_action_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED std_tools tf roscpp mathlib robotlib cds_execution tf cmake_modules )

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_tools tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


set(H_FILES  	${INCLUDE_DIR}/kuka_action_server/base_action.h
                ${INCLUDE_DIR}/kuka_action_server/default_topics.h
                ${INCLUDE_DIR}/kuka_action_server/action_server.h
)

set(SRC_FILES
                ${SRC_DIR}/base_action.cpp
                ${SRC_DIR}/action_server.cpp
)

add_library(${PROJECT_NAME} SHARED ${H_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )

