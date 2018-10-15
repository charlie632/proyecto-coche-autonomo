cmake_minimum_required(VERSION 2.8.3)
project(example_opencv)

find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(LIBRARIES OpenCV
               CATKIN_DEPENDS roscpp)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_executable(showcam src/show_webcam.cpp)
target_link_libraries(showcam ${catkin_LIBRARIES} 
                              ${OpenCV_LIBRARIES})

add_executable(showimage src/show_thisimage.cpp)
target_link_libraries(showimage ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES})

add_executable(segment src/segmentador.cpp)
target_link_libraries(segment ${catkin_LIBRARIES} 
                                ${OpenCV_LIBRARIES})
