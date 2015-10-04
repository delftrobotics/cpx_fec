cmake_minimum_required(VERSION 2.8.3)
project(cpx_fec_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
)

add_service_files(
	DIRECTORY srv
)

generate_messages()

catkin_package(
	CATKIN_DEPENDS message_runtime
	LIBRARIES ${PROJECT_NAME}
)

# Make a dummy library to force dependent targets to have dependencies on message generation.
configure_file("/dev/null" "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_library(${PROJECT_NAME} "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
