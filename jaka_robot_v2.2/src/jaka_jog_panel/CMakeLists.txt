cmake_minimum_required(VERSION 3.0.2)
project(jaka_jog_panel)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pluginlib
  sensor_msgs
  message_generation
  geometry_msgs
)

find_package(Qt5  REQUIRED COMPONENTS  Widgets)
#FIND_PACKAGE (Qt4)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#SET(CMAKE_PREFIX_PATH /opt/Qt5.9.1/5.9/gcc_64)

#set(SOURCES
#  src/jakajogpanel.cpp
#  src/demo_main.cpp
#)

#set(FORMS
#  src/jakajogpanel.ui
#)

set(QT_LIBRARIES Qt5::Widgets)

#set(SOURCE_FILES
#    src/jakajogpanel.cpp
#    ${MOC_FILES}
#    ${ui_FILES}
#)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    std_msgs
#    sensor_msgs
#    geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES jaka_jog_panel
  CATKIN_DEPENDS roscpp rospy std_msgs pluginlib sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
#  include/jaka_jog_panel
#  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src/libs
)

#ADD_LIBRARY(${PROJECT_NAME}
#  src/main_window.cpp
#)

#qt5_wrap_cpp(MOC_FILES src/jakajogpanel.h)

#qt5_wrap_ui(ui_FILES ui/jakajogpanel.ui)



#add_executable(${PROJECT_NAME}jakajogpanel ${SOURCES} ${FORMS} )
#add_executable(jakajogpanel src/jakajogpanel.cpp )

#add_executable(jakajogpanel
               #src/main.cpp
#               src/jakajogpanel.cpp
             # ${MOC_FILES}
              #${ui_FILES})

#target_link_libraries(jakajogpanel  ${SOURCE_FILES})

#target_link_libraries(jakajogpanel  Qt5::Widgets )

#rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/jaka_jog_panel/*.hpp)

#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

#add_executable(jakajogpanel ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(jakajogpanel src/main.cpp src/main_window.cpp ui/main_window.ui ${QT_MOC_HPP})
target_link_libraries(jakajogpanel ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
add_dependencies(jakajogpanel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS jakajogpanel RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(joint_state_subscriber src/joint_state_subscriber.cpp)
target_link_libraries(joint_state_subscriber ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(joint_state_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(joint_state_subscriber src/joint_state_subscriber.cpp)
#target_link_libraries(joint_state_subscriber ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
#add_dependencies(joint_state_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
