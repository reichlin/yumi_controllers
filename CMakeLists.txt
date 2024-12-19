cmake_minimum_required(VERSION 2.8.3)
project(yumi_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_wrapper
  roscpp
  std_msgs
  geometry_msgs
  quest2ros
)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS orocos_kdl system_lib
  CATKIN_DEPENDS  kdl_parser kdl_wrapper roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_executable(kdl_controller src/kdl_controller.cpp)
add_executable(circlerun_pad_controller src/circlerun_pad_controller.cpp)
add_executable(yumi_pushing src/yumi_pushing.cpp)
add_executable(yumi_full_motion src/yumi_full_motion.cpp)
add_executable(yumi_ik_ctrl src/yumi_ik_ctrl.cpp)
add_executable(yumi_vr src/yumi_vr.cpp)
add_executable(yumi_taskspace_ctrl src/yumi_taskspace_ctrl.cpp)
#add_executable(yumi_bimanual_motion src/yumi_bimanual_motion.cpp)
# add_executable(yumi_vr_motion src/yumi_vr_motion.cpp)
target_link_libraries(kdl_controller  ${catkin_LIBRARIES} )
target_link_libraries(circlerun_pad_controller  ${catkin_LIBRARIES} )
target_link_libraries(yumi_pushing  ${catkin_LIBRARIES} )
target_link_libraries(yumi_full_motion  ${catkin_LIBRARIES} )
#target_link_libraries(yumi_bimanual_motion  ${catkin_LIBRARIES} )
# target_link_libraries(yumi_vr_motion  ${catkin_LIBRARIES} )
target_link_libraries(yumi_ik_ctrl  ${catkin_LIBRARIES} )
target_link_libraries(yumi_vr  ${catkin_LIBRARIES} )
target_link_libraries(yumi_taskspace_ctrl  ${catkin_LIBRARIES} )