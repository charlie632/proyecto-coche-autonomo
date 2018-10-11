cmake_minimum_required(VERSION 2.8.3)
project(image_converter)

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

add_executable(image_converter src/ImageConverter.cpp src/main.cpp)

target_link_libraries(image_converter
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
