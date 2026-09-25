cmake_minimum_required(VERSION 3.0.2)
project(autonomous_quadcopter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/Main.py
  scripts/Autonomous_Flight.py
  scripts/Emergency_Failsafe.py
  scripts/GUI_Listener.py
  scripts/Sensor_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
