cmake_minimum_required(VERSION 3.0.2)
project(rc_gimbal_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-Wall -Werror -Wno-enum-compare)
find_package(catkin REQUIRED
  COMPONENTS
  roscpp

  rc_common
  pluginlib
  controller_interface
  effort_controllers
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  angles
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  pluginlib
  rc_common

  effort_controllers
  tf2_eigen
  tf2_geometry_msgs
  dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/gimbal_base.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
        DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
        FILES rc_gimbal_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)