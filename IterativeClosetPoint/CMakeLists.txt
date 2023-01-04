cmake_minimum_required(VERSION 3.5)
project(point_cloud_visualization)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_executable(point_cloud_visualization pcl_visualization.cpp)

target_link_libraries(point_cloud_visualization ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
