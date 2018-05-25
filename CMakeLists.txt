cmake_minimum_required(VERSION 2.8)

project(pcl)

SET(OpenNI2_SO_DIR ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable (pcl ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries (pcl ${PCL_LIBRARIES} ${OpenNI2_SO_DIR}/libOpenNI2.so)
