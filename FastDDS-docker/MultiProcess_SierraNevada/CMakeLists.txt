cmake_minimum_required(VERSION 3.10.2)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(MultiProcessSierraNevada)

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

message(STATUS "Configuring multi-process Sierra Nevada benchmark")
file(GLOB DDS_BENCHMARK_SOURCES_CXX "src/*.cxx" "src/msg/*.cxx")

add_executable(montreal src/montreal.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(montreal PUBLIC include)
target_link_libraries(montreal fastrtps fastcdr)

add_executable(lyon src/lyon.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(lyon PUBLIC include)
target_link_libraries(lyon fastrtps fastcdr)

add_executable(hamburg src/hamburg.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(hamburg PUBLIC include)
target_link_libraries(hamburg fastrtps fastcdr)

add_executable(osaka src/osaka.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(osaka PUBLIC include)
target_link_libraries(osaka fastrtps fastcdr)

add_executable(mandalay src/mandalay.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(mandalay PUBLIC include)
target_link_libraries(mandalay fastrtps fastcdr)

add_executable(ponce src/ponce.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(ponce PUBLIC include)
target_link_libraries(ponce fastrtps fastcdr)

add_executable(barcelona src/barcelona.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(barcelona PUBLIC include)
target_link_libraries(barcelona fastrtps fastcdr)

add_executable(georgetown src/georgetown.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(georgetown PUBLIC include)
target_link_libraries(georgetown fastrtps fastcdr)

add_executable(geneva src/geneva.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(geneva PUBLIC include)
target_link_libraries(geneva fastrtps fastcdr)

add_executable(arequipa src/arequipa.cpp ${DDS_BENCHMARK_SOURCES_CXX})
target_include_directories(arequipa PUBLIC include)
target_link_libraries(arequipa fastrtps fastcdr)

