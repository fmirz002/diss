cmake_minimum_required(VERSION 2.8.3)
project(ur10_gripper_vision)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    geometric_shapes
    roscpp
    tf2_ros
    tf2_sensor_msgs
    tf2
    geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    tf2
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable (filterPoints src/filterPoints.cpp)
target_link_libraries (filterPoints ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS filterPoints DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable (segment_blue src/segment_blue.cpp)
target_link_libraries (segment_blue ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS segment_blue DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable (segment_red src/segment_red.cpp)
target_link_libraries (segment_red ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS segment_red DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable (segment_yellow src/segment_yellow.cpp)
target_link_libraries (segment_yellow ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS segment_yellow DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable (segment_green src/segment_green.cpp)
target_link_libraries (segment_green ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS segment_green DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable (objectPos src/objectPos.cpp)
target_link_libraries (objectPos ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS objectPos DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

