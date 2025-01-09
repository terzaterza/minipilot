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

# Dependencies
set(protobuf_BUILD_TESTS OFF)
add_subdirectory("lib/emblibcpp")
add_subdirectory("lib/protobuf")

# Generate protobuf cpp files from sources
add_subdirectory("protobuf")

# Source files for minipilot
add_library(minipilot STATIC
    src/dynamics/copter/copter.cpp
    src/dynamics/copter/quadcopter.cpp
    src/tasks/task_accelerometer.cpp
    src/tasks/task_gyroscope.cpp
    src/tasks/task_logger.cpp
    src/util/logger.cpp
    src/main.cpp
)

# Header files to be used by minipilot
target_include_directories(minipilot PRIVATE
    "${PROJECT_SOURCE_DIR}/src"
)

# Header files to be used by external porting projects
target_include_directories(minipilot PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

# Link dependency libraries to minipilot
target_link_libraries(minipilot PUBLIC
    emblib
    protobuf::libprotobuf
    minipilot-proto
)

# Testing on the fly
if (PROJECT_IS_TOP_LEVEL)
    add_subdirectory(test)
endif()