cmake_minimum_required(VERSION 3.10.2)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(SierraNevada)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Set C++14
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++14 SUPPORTS_CXX14)
    if(SUPPORTS_CXX14)
        add_compile_options(-std=c++14)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++14")
    endif()
endif()

message(STATUS "Configuring single-process Sierra Nevada benchmark")
file(GLOB DDS_BENCHMARK_SOURCES_CXX "src/*.cxx" "src/msg/*.cxx")

add_executable(sierra_nevada src/sierra_nevada.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(sierra_nevada PUBLIC include)
target_link_libraries(sierra_nevada fastrtps fastcdr)
