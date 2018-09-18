cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_CPP_TEST)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable (region_growing_segmentation region_growing_segmentation.cpp)
target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES})

#CGAL support

find_package(CGAL)

if(CGAL_FOUND)
    include(${CGAL_USE_FILE})
    include(CGAL_CreateSingleSourceCGALProgram)

    create_single_source_cgal_program("boundary.cpp")
else()
message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()