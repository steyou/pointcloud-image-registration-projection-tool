cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(pcl_project_image)
add_executable(pcl_project_image main.cpp)

#PCL
find_package(PCL 1.2 REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(pcl_project_image ${PCL_LIBRARIES})
endif()

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(pcl_project_image ${OpenCV_LIBS})
endif()

# find_package(VTK REQUIRED)
# include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
# include(${VTK_USE_FILE})


# if (PCL_FOUND)
# endif()
# if (OpenCV_FOUND)
# endif()
#OpenCV