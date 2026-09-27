cmake_minimum_required(VERSION 3.10)

project(ReprojectionProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

add_executable(reprojection_app src/main.cpp src/reprojection.cpp)
target_include_directories(reprojection_app PRIVATE include)
