# Used to compile protobuf files on every build

set(PROTO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/protobuf/src)
set(PROTO_OUT_DIR ${PROJECT_SOURCE_DIR}/src/protobuf)

include(${PROJECT_SOURCE_DIR}/lib/protobuf/cmake/protobuf-generate.cmake)
make_directory(${PROTO_OUT_DIR})

add_library(proto_objects OBJECT
    ${PROTO_SOURCE_DIR}/log.proto
    ${PROTO_SOURCE_DIR}/telemetry.proto
    ${PROTO_SOURCE_DIR}/types.proto
)
target_link_libraries(proto_objects PUBLIC libprotobuf)

protobuf_generate(
    TARGET proto_objects
    IMPORT_DIRS ${PROTO_SOURCE_DIR}
    PROTOC_OUT_DIR ${PROTO_OUT_DIR}
)