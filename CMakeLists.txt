cmake_minimum_required(VERSION 3.20)
enable_testing()
project(simbal CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE "Debug")
set(DCMAKE_EXPORT_COMPILE_COMMANDS=1)

add_subdirectory(libs/eigen EXCLUDE_FROM_ALL)
target_compile_definitions(eigen INTERFACE EIGEN_MAX_ALIGN_BYTES=0)

add_subdirectory(libs/Catch2 EXCLUDE_FROM_ALL)
list(APPEND CMAKE_MODULE_PATH libs/Catch2/contrib)

add_subdirectory(src)
add_subdirectory(tests)