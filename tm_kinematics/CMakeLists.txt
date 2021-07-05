cmake_minimum_required(VERSION 2.8.3)
project(tm_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp geometry_msgs moveit_core moveit_ros_planning pluginlib tf_conversions
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tm700_kin tm900_kin tm700_moveit_plugin tm900_moveit_plugin tm1100_kin tm1300_kin tm1100_moveit_plugin tm1300_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning pluginlib tf_conversions
  DEPENDS boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(tm700_kin src/tm_kin.cpp)
set_target_properties(tm700_kin PROPERTIES COMPILE_DEFINITIONS "TM700_DH_PARAMS")

add_library(tm900_kin src/tm_kin.cpp)
set_target_properties(tm900_kin PROPERTIES COMPILE_DEFINITIONS "TM900_DH_PARAMS")

add_library(tm1100_kin src/tm_kin.cpp)
set_target_properties(tm1100_kin PROPERTIES COMPILE_DEFINITIONS "TM1100_DH_PARAMS")

add_library(tm1300_kin src/tm_kin.cpp)
set_target_properties(tm1300_kin PROPERTIES COMPILE_DEFINITIONS "TM1300_DH_PARAMS")

add_library(tm700_moveit_plugin src/tm_moveit_plugin.cpp)
set_target_properties(tm700_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "TM700_DH_PARAMS")
target_link_libraries(tm700_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  tm700_kin)

add_library(tm900_moveit_plugin src/tm_moveit_plugin.cpp)
set_target_properties(tm900_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "TM900_DH_PARAMS")
target_link_libraries(tm900_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  tm900_kin)

add_library(tm1100_moveit_plugin src/tm_moveit_plugin.cpp)
set_target_properties(tm1100_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "TM1100_DH_PARAMS")
target_link_libraries(tm1100_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  tm1100_kin)

add_library(tm1300_moveit_plugin src/tm_moveit_plugin.cpp)
set_target_properties(tm1300_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "TM1300_DH_PARAMS")
target_link_libraries(tm1300_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  tm1300_kin)

#############
## Install ##
#############

install(TARGETS tm700_kin tm900_kin tm700_moveit_plugin tm900_moveit_plugin
  tm1100_kin tm1300_kin tm1100_moveit_plugin tm1300_moveit_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES tm_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
