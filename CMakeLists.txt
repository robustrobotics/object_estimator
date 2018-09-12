cmake_minimum_required(VERSION 2.8.3)
project(object_estimator)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES None_String.srv 
)

add_message_files(
  FILES EndpointState.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  message_runtime
)


install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)
