cmake_minimum_required(VERSION 2.8.3)
project(path_planning_ompl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


find_package(ompl REQUIRED)

include_directories(${ompl_INCLUDE_DIRS})
message("-- ompl version: ${ompl_VERSION}")
message("-- ompl include dirs: ${ompl_INCLUDE_DIRS}")
message("-- catkin include dirs: ${catkin_INCLUDE_DIRS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planning_ompl
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#set( demo_SRC
#        src/demo.cpp
#        )
#
#add_executable( ${PROJECT_NAME} ${demo_SRC} )
#add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${ompl_LIBS})