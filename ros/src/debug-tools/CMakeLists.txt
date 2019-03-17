cmake_minimum_required(VERSION 2.8.3)
project(debug-tools)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy)
