cmake_minimum_required(VERSION 3.0.2)
project(ros_mono_vo_kitti)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_geometry
  image_transport
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Eigen)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MonoVO
  CATKIN_DEPENDS cmake_modules cv_bridge image_geometry image_transport roscpp rospy std_msgs tf
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
)



#############
## Testing ##
#############


add_executable(ros_mono_vo 
	src/mono_vo.cpp
	src/mono_odom.cpp
)
target_link_libraries(ros_mono_vo ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

