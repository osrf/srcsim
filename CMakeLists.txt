cmake_minimum_required(VERSION 2.8.3)
project(srcsim)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)
find_package(ignition-transport2 REQUIRED)

catkin_package()

########################
## Message generation ##
########################


###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)

############
## Common ##
############

# Create the libLogPlugin.so library.
set(log_plugin_name LogPlugin)
add_library(${log_plugin_name} src/LogPlugin.cc)
target_link_libraries(${log_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
#add_dependencies(${log_plugin_name}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
install(TARGETS ${log_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

##########################
## Qualification task 1 ##
##########################

# Create the libQual1Plugin.so library.
set(qual1_plugin_name Qual1Plugin)
add_library(${qual1_plugin_name} src/Qual1Plugin.cc)
target_link_libraries(${qual1_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)
#add_dependencies(${qual1_plugin_name}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
install(TARGETS ${qual1_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the qual1 executable.
set(qual1_binary_name qual1)
add_executable(${qual1_binary_name} src/qual1.cc)
target_link_libraries(${qual1_binary_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

##########################
## Qualification task 2 ##
##########################

# Create the libQual2Plugin.so library.
set(qual2_plugin_name Qual2Plugin)
add_library(${qual2_plugin_name} src/Qual2Plugin.cc)
target_link_libraries(${qual2_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
#add_dependencies(${qual2_plugin_name}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
install(TARGETS ${qual2_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


#############
## Install ##
#############

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
