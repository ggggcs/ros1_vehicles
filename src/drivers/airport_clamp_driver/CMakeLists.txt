cmake_minimum_required(VERSION 3.0.2)
project(airport_clamp_driver)
find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs std_srvs)
add_service_files(FILES SetEndpoints.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs std_srvs)
catkin_install_python(PROGRAMS scripts/airport_clamp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
