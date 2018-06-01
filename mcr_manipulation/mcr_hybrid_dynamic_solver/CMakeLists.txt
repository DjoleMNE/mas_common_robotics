cmake_minimum_required(VERSION 2.8.3)
project(mcr_hybrid_dynamic_solver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  mcr_common_msgs
  mcr_manipulation_utils
  brics_actuator
  interactive_markers
)

find_package(orocos_kdl)
find_package(mcr_manipulation_utils)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    kdl_parser
    mcr_manipulation_utils
    roscpp
    std_msgs
    trajectory_msgs
    sensor_msgs
    mcr_common_msgs
    brics_actuator
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  scripts/${PROJECT_NAME}_node.cpp
  src/hd_solver_vereshchagin.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    ${mcr_manipulation_utils_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mas_dynamic_control.cpp
# )

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}
  #${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mas_dynamic_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
