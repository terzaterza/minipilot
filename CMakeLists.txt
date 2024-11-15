cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(minipilot VERSION 1.0)

# EMBLIB configuration
add_library(emblib_config INTERFACE)
target_include_directories(emblib_config INTERFACE
    "${PROJECT_SOURCE_DIR}/src"
)


# Protobuf
set(ABSL_ENABLE_INSTALL ON)
set(ABSL_BUILD_TESTING OFF)
set(protobuf_BUILD_TESTS OFF)


add_subdirectory("lib/emblibcpp")
add_subdirectory("lib/abseil-cpp")
add_subdirectory("lib/protobuf")

# Used to compile protobuf files on every build
include(lib/protobuf/cmake/protobuf-generate.cmake)
add_library(proto_objects OBJECT
    protobuf/types.proto
    protobuf/log.proto
    protobuf/telemetry.proto
)
target_link_libraries(proto_objects PUBLIC libprotobuf)
protobuf_generate(
    TARGET proto_objects
    IMPORT_DIRS ${PROJECT_SOURCE_DIR}/protobuf
    PROTOC_OUT_DIR ${PROJECT_SOURCE_DIR}/src/protobuf
)

add_library(minipilot STATIC
    src/dynamics/copter/copter.cpp
    src/dynamics/copter/quadcopter.cpp
    src/mp_main.cpp
)

# Header files to be used by minipilot
target_include_directories(minipilot PRIVATE
    "${PROJECT_SOURCE_DIR}/src"
)

# Header files to be used by external porting projects
target_include_directories(minipilot PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(minipilot PUBLIC
    emblib
    libprotobuf
)

# Executable for testing on the fly
if (PROJECT_IS_TOP_LEVEL)
    # Flags for the build
    set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2")

    # Executable for running custom testing and debugging
    add_executable(minipilot_test test.cpp)
    target_link_libraries(minipilot_test PRIVATE minipilot)
endif()