cmake_minimum_required(VERSION 2.8.3)

project(snavquad_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
  quadrotor_msgs
  nav_msgs
  nodelet
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs nodelet tf_conversions
  DEPENDS
  LIBRARIES
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(posNode src/posNode.cpp)
target_link_libraries(posNode
	${catkin_LIBRARIES}
	snav_arm
)

add_executable(vioOdom src/vioOdom.cpp)

target_link_libraries(vioOdom
	${catkin_LIBRARIES}
	snav_arm
)

add_library(so3cmd_to_snav_nodelet src/so3cmd_to_snav_nodelet.cpp)
add_dependencies(so3cmd_to_snav_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(so3cmd_to_snav_nodelet ${catkin_LIBRARIES} snav_arm)

add_library(trpycmd_to_snav_nodelet src/trpycmd_to_snav_nodelet.cpp)
add_dependencies(trpycmd_to_snav_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(trpycmd_to_snav_nodelet ${catkin_LIBRARIES} snav_arm)

install(TARGETS posNode vioOdom so3cmd_to_snav_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
