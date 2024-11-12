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
set(protobuf_BUILD_TESTS OFF)
set(protobuf_ABSL_PROVIDER package)
LIST(APPEND CMAKE_PROGRAM_PATH "lib/abseil-cpp")
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)


add_subdirectory("lib/emblibcpp")
add_subdirectory("lib/abseil-cpp")
add_subdirectory("lib/protobuf")


add_library(minipilot STATIC
    src/dynamics/copter/copter.cpp
    src/dynamics/copter/quadcopter.cpp
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
