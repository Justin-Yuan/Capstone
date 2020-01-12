cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge 
		image_transport 
		sensor_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(contest3 src/contest3.cpp
			src/imageTransporter.cpp)
target_link_libraries(contest3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
