cmake_minimum_required(VERSION 3.0.2)
project(spot_collector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_py
  tf2_sensor_msgs
  ros_numpy
  depth_image_proc
  std_srvs
  spot_msgs
  pcl_ros
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

add_service_files(
  FILES
  MultiGrasp.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spot_collector
#  CATKIN_DEPENDS rospy std_msgs
  DEPENDS Eigen
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/cloud2grid.py
    scripts/Interf_real_time.py
    scripts/move_spot_pose.py
    scripts/pick_place_srv.py
    scripts/yolo_garbage_spot.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(lidar_filter src/lidar_filter.cpp)
target_link_libraries(lidar_filter ${catkin_LIBRARIES})

add_executable(traversability_builder src/traversability_builder.cpp)
target_link_libraries(traversability_builder ${catkin_LIBRARIES})
