cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcl_project_image)
add_executable(${PROJECT_NAME} main.cpp)

#PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
message("pcl lib is ${PCL_LIBRARIES}")

find_package(OpenCV 4.6 REQUIRED)
# if (OpenCV_FOUND)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})