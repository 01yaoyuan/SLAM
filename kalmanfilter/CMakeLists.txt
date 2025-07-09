cmake_minimum_required(3.28)
project(KF)
set(CMAKE_CXX_STANDARD 14)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(./include)

aux_source_directory(. SRC_LIST)

add_executable(KF ${SRC_LIST})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)