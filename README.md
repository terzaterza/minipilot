# Minipilot

Minipilot is a small flight controller that supports multiple dynamics models (quadcopter, hexacopter, fixed wing, etc.) which can also be changed mid-flight (WIP). Code in this core repository is written with a custom driver library [emblibcpp]() which allows it to be platform independent. Minipilot is then compiled as a static libary that can be ported to a desired architecture by providing the entry point function with driver implementations for that specific platform.

This provides a nice way to test the firmware within a simulator, by just providing drivers that talk to the simulator socket instead of the actual hardware. Unity based simulator with the Minipilot port which runs on POSIX systems can be found at [minipilot-sim]().

## Project structure
Source files are split between `src` and `include` folders, where files in the `src` folder are used only within the Minipilot itself, but the files in the `include` folder are meant to be used when porting this core library to a specific platform.

Dependencies are located in the `lib` folder, usually as a git submodule, but they can be replaced with symlinks during local development.

Protobuf source files are located in `protobuf` and are compiled during build to `src/protobuf` for internal use, but can also be compiled manually for external use.

Documents describing the system as a whole, but also smaller part in more detail can be found in `docs`. [Overview]() document should be used as a starting point for understanding the architecture of the software.

## Build
This project is configured with a (currently) simple [CMake file](CMakeLists.txt). There exists a "dummy" executable, [test](test/test.cpp), which is only built if this is a top level project, ie. if you are not porting this anywhere. This is used to check whether the library compiles and if the build system is setup correctly.

Requirements for building are gcc and cmake >= 3.13.

Dependencies can be fetched with:
```
git submodule update --init --recursive
```

To compile Minipilot, you can do for example:
```sh
cmake -S . -B build
cmake --build build
```

If the build is successful, should have a `build/test/minipilot_test` executable.

## Porting Minipilot
Minipilot is compiled as a CMake static libary, meaning it does not run on its own. Entry point of the library is the function `mp::mp_main` declared in [mp_main](include/minipilot/mp_main.hpp). It takes in a single parameter, a struct of device drivers for all devices that the library might use.

To use Minipilot on a specific platform, you would create a standard CMake project with an executable. Somewhere in the CMake file, you would add this project as a subdirectory:
```CMake
add_subdirectory("<path-to-project-directory>/minipilot")
```
After adding the executable, you would link the Minipilot library to that executable:
```CMake
target_link_libraries(<executable-name> PRIVATE/PUBLIC minipilot)
```
For a simple example check the `test` folder or the [minipilot-sim]() for a more realistic approach.