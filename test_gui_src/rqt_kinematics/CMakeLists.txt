cmake_minimum_required(VERSION 3.0.2)
project(rqt_kinematics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_kinematics
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/rqt_kinematics
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
