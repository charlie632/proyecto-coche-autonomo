cmake_minimum_required(VERSION 2.8.3)
project(follow_bot)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(follower src/Follower.cpp src/main.cpp)

target_link_libraries(follower
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

#k
