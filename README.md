# Minipilot

Minipilot is a small flight controller that supports multiple vehicle models: quadcopters, hexacopter, fixed-wing v-tails, t-tails, etc. Code in this core repository is written with a custom driver library [emblib](https://github.com/terzaterza/emblib) which allows it to be platform independent. Minipilot is then compiled as a static libary that can be ported to a desired architecture by providing the entry point function with driver implementations for that specific platform.

This provides a nice way to test the firmware within a simulator, by just providing drivers that talk to the simulator socket instead of the actual hardware. Unity based simulator with the Minipilot port which runs on POSIX systems can be found at [minipilot-sim](https://github.com/terzaterza/minipilot-sim).

## Project structure
Source files can be found in `src` and `include` folders, where files in the `src` folder are used only within the Minipilot itself (included as private in the CMake project), and the files in the `include` folder are meant to be used when porting this core library to a specific platform.

Dependencies are located in the `lib` folder, usually as a git submodule, but they can be replaced with symlinks during local development.

Protobuf source files are located in `protobuf` and are compiled during build (or manually) to `protobuf/out`. C++ protobuf output is then built as a static library which is linked to minipilot.

Documents describing the system as a whole, but also smaller parts in more detail can be found in `docs`. [Overview](docs/Overview.md) document should be used as a starting point for understanding the architecture of the software.

Python notebooks which are used for formula derivations or signal analysis are found in the `python` folder. This folder has a [requirements.txt](python/requirements.txt) which can be used to install (`pip install -r requirements.txt`) all needed pip dependencies for running the scripts/notebooks.

## Build
This project is configured with a (currently) simple [CMake file](CMakeLists.txt).
Requirements for building are **gcc** and **cmake** >= 3.13.

Dependencies can be fetched with:
```
git submodule update --init --recursive
```

To compile Minipilot, you can do for example:
```sh
cmake -S . -B build
cmake --build build
```

If the build is successful, should have a `build/libminipilot.a` static library.

## Porting Minipilot
Minipilot is compiled as a CMake static libary, meaning it does not run on its own. Entry point of the library is the function `mp::main` declared in [main.hpp](include/mp/main.hpp). It takes in a struct of device drivers for all devices that the library might use, as well as the vehicle model and the state estimator which are to be used.

To use Minipilot on a specific platform, you would create a standard CMake project with an executable and add this project as a subdirectory:
```CMake
add_subdirectory("<path-to-project-directory>/minipilot")
```
After adding the executable, you would link the minipilot library to that executable:
```CMake
target_link_libraries(<executable-name> PRIVATE/PUBLIC minipilot)
```
For an example check [minipilot-sim](https://github.com/terzaterza/minipilot-sim).
