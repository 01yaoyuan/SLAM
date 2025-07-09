cmake_minimum_required(VERSION 3.8)
project(lm)

set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.12 REQUIRED COMPONENTS common io registration)
find_package(Eigen3 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(../include)

aux_source_directory(. SRC_LIST)

message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")

link_directories(${PCL_LIBRARY_DIRS})

add_executable(main ${SRC_LIST})
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
target_link_libraries(main ${PCL_LIBRARIES} ${PCL_IO_LIBRARY})