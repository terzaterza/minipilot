# Used to compile protobuf files on every build

# Path to the build protobuf compiler
set(PROTOC ${CMAKE_CURRENT_BINARY_DIR}/../lib/protobuf/protoc)

set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROTO_CPP_OUT ${CMAKE_CURRENT_SOURCE_DIR}/out/cpp/pb)
set(PROTO_PY_OUT ${CMAKE_CURRENT_SOURCE_DIR}/out/py)

make_directory(${PROTO_CPP_OUT})
make_directory(${PROTO_PY_OUT})


# Get all protobuf src files
file(GLOB_RECURSE PROTO_SRCS ${PROTO_SRC_DIR}/*.proto)

# List of all generated cpp files (1 for each proto source)
set(PROTO_CC_SRCS "")
foreach(_SRC_ABSPATH ${PROTO_SRCS})
    # Replace the .proto extension in the filename with .pb.cc
    file(RELATIVE_PATH _SRC_RELPATH ${PROTO_SRC_DIR} ${_SRC_ABSPATH})
    string(REGEX REPLACE "[.]proto$" ".pb.cc" _CC_RELPATH ${_SRC_RELPATH})
    set(_CC_ABSPATH ${PROTO_CPP_OUT}/${_CC_RELPATH})

    # Generate output files from this proto source
    add_custom_command(
        OUTPUT ${_CC_ABSPATH}
        DEPENDS ${_SRC}
        COMMAND ${PROTOC} --cpp_out=${PROTO_CPP_OUT} --python_out=${PROTO_PY_OUT} -I=${PROTO_SRC_DIR} ${_SRC_ABSPATH}
    )
    # Add this cpp files to the list to be used by the static library
    list(APPEND PROTO_CC_SRCS ${_CC_ABSPATH})
endforeach()

# Static library with all the generated cpp files
add_library(minipilot-proto STATIC ${PROTO_CC_SRCS})
target_link_libraries(minipilot-proto PUBLIC libprotobuf)
target_include_directories(minipilot-proto PUBLIC ${PROTO_CPP_OUT}) # Needed since protobuf includes don't have relative paths
target_include_directories(minipilot-proto PUBLIC ${PROTO_CPP_OUT}/..)
add_dependencies(minipilot-proto protoc)