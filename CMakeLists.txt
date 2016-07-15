
cmake_minimum_required(VERSION 2.8.3)
project(hanp_head_behavior)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  hanp_msgs
  nav_msgs
  roscpp
  tf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/HANPHeadBehavior.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  # LIBRARIES
  #   hanp_local_planner
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    hanp_msgs
    nav_msgs
    roscpp
    tf
    visualization_msgs
#   DEPENDS
#     system_lib
)



## build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${EIGEN_DEFINITIONS})

# declare a c++ executable
add_executable(hanp_head_behavior
  src/hanp_head_behavior.cpp
)

# cmake target dependencies of the c++ executable
add_dependencies(hanp_head_behavior ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ executable against
target_link_libraries(hanp_head_behavior ${catkin_LIBRARIES})



## install ##

# executables and/or libraries for installation
install(TARGETS hanp_head_behavior
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# other files for installation (launch, data, bag files)
foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



## testing ##

## add catch based cpp test target
# catkin_add_catchtest(${PROJECT_NAME}-test test/test_hanp_head_behavior.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## add nosetest file folders
# catkin_add_nosetests(test)
