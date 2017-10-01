cmake_minimum_required(VERSION 2.8.3)
project(tese_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tese
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/bringup.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


