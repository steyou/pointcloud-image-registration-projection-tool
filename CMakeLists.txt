cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC  -lm -Wl,--no-as-needed -DLINUX -g -Wall \
-std=c++11 -Wunused-variable -Wwrite-strings -Wsign-compare -fpermissive ")
set(CMAKE_BUILD_TYPE "Release")

project(pcl_project_image)


INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(./include)
AUX_SOURCE_DIRECTORY(./ gSOURCE_FILES_)


add_executable(${PROJECT_NAME} "main.cpp"
    ${gSOURCE_FILES_}
    )
    
    
#PCL
find_package(PCL 1.2 REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
    set(OpenCV_LIBS ${OpenCV_LIBRARIES})
    MESSAGE(STATUS "The Opencv's include directory is:" ${OpenCV_INCLUDE_DIRS})
    MESSAGE(STATUS "The Opencv's OpenCV_LIBS is:" ${OpenCV_LIBS})
endif()



# find_package(VTK REQUIRED)
# include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
# include(${VTK_USE_FILE})


# if (PCL_FOUND)
# endif()
# if (OpenCV_FOUND)
# endif()
#OpenCV