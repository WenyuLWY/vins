cmake_minimum_required(VERSION 3.0.2)
project(vins)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  tf
  cv_bridge
  image_transport
  rosbag
  

  pcl_conversions
  pcl_ros

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  include/vins_estimator
  include/feature_tracker
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# ${CERES_INCLUDE_DIRS}
catkin_package(
)


set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX)

# # feature tracker
add_library(feature_tracker_nodelet src/feature_tracker/feature_tracker_nodelet.cpp)
target_link_libraries(feature_tracker_nodelet ${COMMON_LIBRARIES})

# add_executable(feature_tracker_node src/feature_tracker/feature_tracker_node.cpp)
# target_link_libraries(feature_tracker_node ${COMMON_LIBRARIES})

# add_executable(feature_tracker_test src/feature_tracker/feature_tracker_test.cpp)
# target_link_libraries(feature_tracker_test ${COMMON_LIBRARIES})

# # object_detection
# add_executable(depth_cluster_node src/object_detection/depth_cluster.cpp)
# target_link_libraries(depth_cluster_node ${COMMON_LIBRARIES})

# object_detection nodelet
add_library(object_detection_nodelet src/object_detection/object_detection_nodelet.cpp)
target_link_libraries(object_detection_nodelet ${COMMON_LIBRARIES})

# add_executable(vins_estimator_node src/vins_estimator/vins_estimator_node.cpp)
# target_link_libraries(vins_estimator_node ${COMMON_LIBRARIES})

# add_executable(single_test src/feature_tracker/single_image_test.cpp)
# target_link_libraries(single_test ${COMMON_LIBRARIES})


# add_executable(vins_node src/rosNodeTest.cpp)
# target_link_libraries(vins_node ${COMMON_LIBRARIES})
 

