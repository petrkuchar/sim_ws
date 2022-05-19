cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  rospy
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  pcl_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ref src/trajectory.cpp)
add_executable(pp src/pure_pursuit.cpp)
add_executable(linearmpc src/MPC.cpp)





catkin_install_python(PROGRAMS scripts/line.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(pp ${catkin_LIBRARIES})
target_link_libraries(mpc ${catkin_LIBRARIES})
target_link_libraries(ref ${catkin_LIBRARIES})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(osqp REQUIRED)

find_package(OsqpEigen)


target_link_libraries (ref Eigen3::Eigen)
target_link_libraries(mpc osqp::osqpstatic)
target_link_libraries(mpc OsqpEigen::OsqpEigen)

