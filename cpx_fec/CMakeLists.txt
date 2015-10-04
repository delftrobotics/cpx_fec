cmake_minimum_required(VERSION 2.8.3)
project(cpx_fec)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	dr_base
	modbus
	cpx_fec_msgs
)

catkin_package()

dr_include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
