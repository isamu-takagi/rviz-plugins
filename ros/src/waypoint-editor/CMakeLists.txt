cmake_minimum_required(VERSION 2.8.3)
project(waypoint-editor)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  tf2
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  geometry_msgs
  pcl_ros
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

qt5_wrap_cpp(QT_HEADER_FILES
  src/event_capture_server.hpp
  src/event_capture_client.hpp
  src/event_capture_tool.hpp
  src/point_cloud_map.hpp
  src/waypoint_editor_library.hpp
  src/waypoint_editor_marker.hpp
  src/waypoint_editor_panel.hpp
)

set(QT_SOURCE_FILES
  src/event_capture_server.cpp
  src/event_capture_client.cpp
  src/event_capture_tool.cpp
  src/point_cloud_map.cpp
  src/waypoint_editor_library.cpp
  src/waypoint_editor_marker.cpp
  src/waypoint_editor_panel.cpp
)

add_library(${PROJECT_NAME}
  ${QT_HEADER_FILES}
  ${QT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
