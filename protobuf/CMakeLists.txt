# Used to compile protobuf files on every build

include(${PROJECT_SOURCE_DIR}/lib/protobuf/cmake/protobuf-generate.cmake)

add_library(proto_objects OBJECT
    ${PROJECT_SOURCE_DIR}/protobuf/types.proto
    ${PROJECT_SOURCE_DIR}/protobuf/log.proto
    ${PROJECT_SOURCE_DIR}/protobuf/telemetry.proto
)
target_link_libraries(proto_objects PUBLIC libprotobuf)

protobuf_generate(
    TARGET proto_objects
    IMPORT_DIRS ${PROJECT_SOURCE_DIR}/protobuf
    PROTOC_OUT_DIR ${PROJECT_SOURCE_DIR}/src/protobuf
)