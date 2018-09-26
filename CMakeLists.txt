cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)
project(THESIS VERSION 1.0.0 DESCRIPTION "Dynamic Library for thesis project")
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")

#==========================
# Supporting packages
#==========================
set(docopt_SOURCES docopt/docopt.cpp)
set(docopt_HEADERS
		docopt/docopt.h
		docopt/docopt_private.h
		docopt/docopt_util.h
		docopt/docopt_value.h
        )
add_library(docopt_s STATIC ${docopt_SOURCES} ${docopt_HEADERS})

#===========================
# Compile main packages
#===========================
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(pcd_write_test pcd_write.cpp)
#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

#CGAL support
add_library(CGAL STATIC IMPORTED COMPONENTS Core)
find_package(CGAL QUIET COMPONENTS Core 4.12 REQUIRED)

if(CGAL_FOUND)
    include(${CGAL_USE_FILE})
    include(CGAL_CreateSingleSourceCGALProgram)

    add_library(BoundaryLib STATIC boundary.cpp)
    target_link_libraries(BoundaryLib ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
    #create_single_source_cgal_program("boundary.cpp")
else()
message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

#add_executable (boundary_test boundary_test.cpp)
#target_link_libraries (boundary_test ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} BoundaryLib)

#add_executable (region_growing_segmentation region_growing_segmentation.cpp)
#target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} BoundaryLib)

add_library(SegmentorLib STATIC segmentor.cpp)
target_link_libraries (SegmentorLib ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} BoundaryLib)

add_executable (thesis thesis.cpp)
target_link_libraries (thesis ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} SegmentorLib docopt_s)

#add_executable (segmentor_test segmentor_test.cpp)
#target_link_libraries (segmentor_test ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} SegmentorLib)

# add_library(SegmentorLib SHARED segmentor.cpp)
# target_link_libraries (SegmentorLib ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} BoundaryLib)
# set_target_properties(SegmentorLib PROPERTIES VERSION ${PROJECT_VERSION})
# set_target_properties(SegmentorLib PROPERTIES SOVERSION 1)
# set_target_properties(SegmentorLib PROPERTIES PUBLIC_HEADER include/segmentor.h)

# include(GNUInstallDirs)
# install(TARGETS SegmentorLib
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# configure_file(SegmentorLib.pc.in SegmentorLib.pc @ONLY)
# install(FILES ${CMAKE_BINARY_DIR}/SegmentorLib.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)