cmake_minimum_required(VERSION 2.8.3)
project(in2_ibeo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mrpt_bridge
  roscpp
  std_msgs
  rospy
  cv_bridge
  image_transport
  in2_msgs

#  pcl_conversions
 # pcl_ros
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(MRPT REQUIRED base slam hwdrivers)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrpt_ros_tutorial
#  CATKIN_DEPENDS mrpt_navigation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(ibeo_4L_node src/start_ibeo_4L.cpp src/CIbeoLuxETH.cpp src/hwdrivers-precomp.cpp)
target_link_libraries(ibeo_4L_node
  ${catkin_LIBRARIES}
  ${MRPT_LIBS}

)

add_executable(ibeo_8L_node src/start_ibeo_8L.cpp src/CIbeoLuxETH.cpp src/hwdrivers-precomp.cpp)
target_link_libraries(ibeo_8L_node
  ${catkin_LIBRARIES}
  ${MRPT_LIBS}

)

add_executable(ibeo_fusion_node src/ibeo_fusion.cpp)
target_link_libraries(ibeo_fusion_node
  ${catkin_LIBRARIES}
#  ${MRPT_LIBS}

)

add_executable(heightmap_g20_node src/heightmap_node.cpp src/CHeightmap_g20.cpp)
target_link_libraries(heightmap_g20_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

