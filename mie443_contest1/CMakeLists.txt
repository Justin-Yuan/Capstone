cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# reference run 
# add_executable(reference src/reference.cpp src/planners.cpp)
# target_link_libraries(reference ${catkin_LIBRARIES} ${OpenCV_LIB})

# real contest1 run 
add_executable(contest1 src/contest1.cpp src/planners.cpp)
target_link_libraries(contest1 ${catkin_LIBRARIES} ${OpenCV_LIB})

# occupancy grid trial 
add_executable(ros_occupancy_grid src/ros_occupancy_grid.cpp)
target_link_libraries(ros_occupancy_grid ${catkin_LIBRARIES} ${OpenCV_LIB})
