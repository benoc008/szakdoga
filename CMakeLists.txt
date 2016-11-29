cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(old)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(LibLAS REQUIRED)
include_directories(${LIBLAS_INCLUDE_DIR})
link_directories(${LIBLAS_LIBRARIES})
add_definitions(${LIBLAS_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable (old main.cpp CloudUtil.hpp CloudHelper.hpp ClusteringResults.hpp LasToPcdConverter.hpp)
target_link_libraries (old ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})
target_compile_features(old PRIVATE cxx_range_for)