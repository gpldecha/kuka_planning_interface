cmake_minimum_required(VERSION 2.8.3)
project(pour_kuka)

find_package(catkin REQUIRED std_tools tf roscpp mathlib robotlib cds_execution
tf cmake_modules kuka_action_server kuka_action_client)



set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")



## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS std_tools tf
)


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})



# --- Create Action Server Node Pour action

add_executable(pour_server_node  ${INCLUDE_DIR}/pour_kuka/pour_action_server.h 
 				 ${SRC_DIR}/pour_action_server.cpp
				 ${SRC_DIR}/pour_action_server_node.cpp	)
target_link_libraries(pour_server_node ${catkin_LIBRARIES})

# --- Create Action Client Node Pour action

add_executable(pour_client_node ${INCLUDE_DIR}/pour_kuka/pour_client.h
                                ${SRC_DIR}/pour_client.cpp
                                ${SRC_DIR}/pour_client_node.cpp)
target_link_libraries(pour_client_node ${catkin_LIBRARIES})




