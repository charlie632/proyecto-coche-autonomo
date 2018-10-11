cmake_minimum_required(VERSION 2.8.3)
project(control_coche)

find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS ackermann_msgs cv_bridge
  image_transport roscpp rospy sensor_msgs std_msgs)


catkin_package( 
  LIBRARIES OpenCV
  CATKIN_DEPENDS ackermann_msgs cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)


include_directories(${OpenCV_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})


add_executable(car_teleop src/car_teleop.cpp)
target_link_libraries(car_teleop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
