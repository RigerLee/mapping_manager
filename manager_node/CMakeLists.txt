cmake_minimum_required(VERSION 2.8.3)
project(mapping_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    camera_model
    cv_bridge
    roslib
    octomap_ros
    octomap_msgs
    message_filters
    )

find_package(OpenCV)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include_directories("include")

catkin_package()

set(mm_lib
    src/manager.cpp
    src/keyframe.cpp)

# Create global mapping manager
add_executable(global_manager ${mm_lib} src/manager_global_node.cpp)
target_link_libraries(global_manager ${catkin_LIBRARIES}  ${OpenCV_LIBS})

# Create local mapping manager
#add_executable(local_manager ${mm_lib} src/manager_local_node.cpp)
#target_link_libraries(local_manager ${catkin_LIBRARIES}  ${OpenCV_LIBS})

