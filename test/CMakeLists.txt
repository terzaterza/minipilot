message(STATUS "Configuring the test port executable")

# Flags for the build
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Executable for running custom testing and debugging
add_executable(minipilot_test test.cpp)
target_link_libraries(minipilot_test PRIVATE minipilot)