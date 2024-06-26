cmake_minimum_required(VERSION 3.6)
project(omniveyor_common)

set(CMAKE_CXX_STANDARD 17)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  move_base_msgs
  roscpp
  tf
  tf2_ros
  urdf
  xacro
)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# # # # ############################################
# # Declare ROS messages, services and actions # #
# # # # ############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
add_message_files(
    FILES
    electricalStatus.msg
    payloadStatus.msg
    moduleEvaluate.msg
    ReFrESHtelemetry.msg
    ReFrESHrequest.msg
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  moduleEstimate.srv
  moduleControl.srv
)

# # Generate actions in the 'action' folder
add_action_files(
  FILES
  LowLevelPose.action
  LowLevelTwist.action
  LowLevelWrench.action
  GoalTrigger.action
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    move_base_msgs
)

# # # # ############################################
# # Declare ROS dynamic reconfigure parameters # #
# # # # ############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

# # # # ###############################
# # catkin specific configuration # #
# # # # ###############################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  roscpp
  nav_msgs
  std_msgs
  move_base_msgs
  tf
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(TFfromOdom_node
  src/tf_from_odom.cpp
)

target_link_libraries(TFfromOdom_node
  ${catkin_LIBRARIES}
)

# # # # #########
# # Install # #
# # # # #########

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

foreach(dir meshes urdf)
  install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# # # # #########
# # Testing # #
# # # # #########

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcv_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
