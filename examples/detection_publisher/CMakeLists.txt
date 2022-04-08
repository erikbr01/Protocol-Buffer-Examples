# Minimum CMake required
cmake_minimum_required(VERSION 3.14.4)

# Project
project(detection_publisher VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Find required protobuf package
find_package(Protobuf REQUIRED)

add_executable(${PROJECT_NAME} detection_publisher.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC addressbook ${Protobuf_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC detection_msg ${Protobuf_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${Protobuf_INCLUDE_DIRS})

find_package(cppzmq)
target_link_libraries(detection_publisher PUBLIC cppzmq)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

target_link_libraries(${PROJECT_NAME} PRIVATE fastcdr fastrtps comm_lib)