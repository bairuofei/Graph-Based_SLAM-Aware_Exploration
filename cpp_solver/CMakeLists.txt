cmake_minimum_required(VERSION 3.0.0)
project(cpp_solver)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

## Build ROS exploration node. Default to true
## Change from command line with argument -DROS=0
SET(ROS TRUE)
SET(ROS TRUE CACHE BOOL "ROS option")


## Find dependecies

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package( OpenCV )
if (OpenCV_FOUND)
  add_definitions ( -DOPENCV_LIBRARY_FOUND=12 )
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	nav2d_navigator
	tsp_planner
	message_generation
)

add_message_files(
  FILES
  PoseGraph.msg
  EdgeDistance.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ExplorationButton.srv
  TspPathList.srv
  RequestGraph.srv
  ReliableLoop.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	LIBRARIES CppExplorationPlugins
	CATKIN_DEPENDS
	roscpp
	rospy
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	nav2d_navigator
	tsp_planner
  message_runtime
)

###########
## Build ##
###########

message("project_name ${PROJECT_NAME}")

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)
add_library(CppExplorationPlugins
  src/MyPlanner.cpp
  src/exploration_plugins.cpp
  src/graph/graph.cpp)

add_library(gridastar STATIC
  src/AStar/AStar2.cpp
  src/AStar/AStar2.h
)

target_compile_options(gridastar PRIVATE -fPIC)

add_dependencies(CppExplorationPlugins ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

target_include_directories(CppExplorationPlugins PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/graph)

# target_link_libraries(CppExplorationPlugins cpp_solver_library ${catkin_LIBRARIES})
target_link_libraries(CppExplorationPlugins ${catkin_LIBRARIES} gridastar pthread)

#############
## Install ##
#############

install(TARGETS CppExplorationPlugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  exploration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/tsp.py
   scripts/exploration_button_client.py
   scripts/exploration_button_server.py
   scripts/path_planner.py
   scripts/utils.py
   scripts/offline_tsp_evaluation.py
   scripts/read_drawio_to_nx.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(pub_path src/pubPathGT.cpp)
target_link_libraries(pub_path ${catkin_LIBRARIES})

add_executable(updateDistance src/updateDistance.cpp)
target_link_libraries(updateDistance ${catkin_LIBRARIES} gridastar)




# message(STATUS "Project Name haha: ${PROJECT_NAME}")
