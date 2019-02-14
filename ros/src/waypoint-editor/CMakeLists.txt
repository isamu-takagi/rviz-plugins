cmake_minimum_required(VERSION 2.8.3)
project(waypoint-editor)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rviz)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

qt5_wrap_cpp(QT_HEADER_FILES
  src/event_capture.hpp
  src/waypoint_editor.hpp
)

set(QT_SOURCE_FILES
  src/event_capture.cpp
  src/waypoint_editor.cpp
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
