cmake_minimum_required(VERSION 2.8.3)
project(nhk2018_main_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  tf2
  can_msgs
#  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
  INCLUDE_DIRS include
#  LIBRARIES nhk2018_main_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
#  CATKIN_DEPENDS message_runtime
#  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nhk2018_main_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_tr_main src/tr_main.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_tr_main ${catkin_LIBRARIES})

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nhk2018_main_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(${PROJECT_NAME}_tr_main_joy src/tr_main_joy.cpp)
target_link_libraries(${PROJECT_NAME}_tr_main_joy ${catkin_LIBRARIES})

add_executable(cr_main_joy src/cr_main_joy.cpp)
target_link_libraries(cr_main_joy ${catkin_LIBRARIES})

add_executable(cr_main_semiauto_2v0 src/cr_main_semiauto_2v0.cpp)
target_link_libraries(cr_main_semiauto_2v0 ${catkin_LIBRARIES})

add_executable(tr_main_auto_1v1 src/tr_main_auto_1v1.cpp)
target_link_libraries(tr_main_auto_1v1 ${catkin_LIBRARIES})

add_executable(tr_main_auto_1v2 src/tr_main_auto_1v2.cpp)
target_link_libraries(tr_main_auto_1v2 ${catkin_LIBRARIES})

add_executable(tr_main_auto_2v0 src/tr_main_auto_2v0.cpp)
target_link_libraries(tr_main_auto_2v0 ${catkin_LIBRARIES})

add_executable(tr_launcher_unitTest src/tr_launcher_unitTest.cpp)
target_link_libraries(tr_launcher_unitTest ${catkin_LIBRARIES})

add_executable(tr_can src/tr_can.cpp)
target_link_libraries(tr_can ${catkin_LIBRARIES})
add_dependencies(tr_can ${catkin_EXPORTED_TARGETS})

add_executable(cr_can src/cr_can.cpp)
target_link_libraries(cr_can ${catkin_LIBRARIES})
add_dependencies(cr_can ${catkin_EXPORTED_TARGETS})

add_executable(usb_can_node src/usb_can_node.cpp)
target_link_libraries(usb_can_node ${catkin_LIBRARIES})
add_dependencies(usb_can_node ${catkin_EXPORTED_TARGETS})




