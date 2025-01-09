# This file is used as a backup if there are problems with the CMake build

ifndef PROTOC
PROTOC := ../build/lib/protobuf/protoc
endif

SRC_DIR = ./src
CPP_OUT_DIR = ./out/cpp/pb # pb suffix for cpp includes
PY_OUT_DIR = ./out/py

SRCS := $(shell find $(SRC_DIR) -name '*.proto')

.PHONY: all
all: cpp py

# Make these build not PHONY
.PHONY: cpp
cpp:
	mkdir -p $(CPP_OUT_DIR)
	$(PROTOC) -I=$(SRC_DIR) --cpp_out=$(CPP_OUT_DIR) $(SRCS)

.PHONY: py
py:
	mkdir -p $(PY_OUT_DIR)
	$(PROTOC) -I=$(SRC_DIR) --python_out=$(PY_OUT_DIR) $(SRCS)