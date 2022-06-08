cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcl_project_image)
add_executable(${PROJECT_NAME} main.cpp)



#PCL
find_package(PCL 1.2 REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
    message("pcl lib is ${PCL_LIBRARIES}")
endif()

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
    message("cv lib is ${OpenCV_LIBS}")
endif()
add_subdirectory(libcbdetect)
target_include_directories(${PROJECT_NAME} PUBLIC libcbdetect/include)
target_link_directories(${PROJECT_NAME} PUBLIC libcbdetect/src/libcbdetect)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/libcbdetect/lib/libcbdetect.so)