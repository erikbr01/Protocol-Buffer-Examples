# Minimum CMake required
cmake_minimum_required(VERSION 3.14.4)

# Project
project(protobuf-cpp-apps)

# Set apps list
set(apps detection_publisher)

# Make sure the generated proto header and source files are visible to the other projects
include_directories(${CMAKE_BINARY_DIR}/protos)

foreach(app ${apps})
    add_subdirectory(${app})
endforeach()
