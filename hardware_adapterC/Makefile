CC = gcc
# Include directories
# Add current directory to include path for local mavlink headers
INCLUDE_DIRS = -I.
CFLAGS = -Wall -Wextra -O2 -std=c11 -pthread $(INCLUDE_DIRS)
LDFLAGS = -pthread -lzmq -lm

# Debug flags
DEBUG_CFLAGS = -Wall -Wextra -g3 -O0 -std=c11 -pthread -DDEBUG $(INCLUDE_DIRS)
DEBUG_LDFLAGS = -pthread -lzmq -lm -g3

# Directories
SRC_DIR = .
OBJ_DIR = obj
OBJ_DIR_DEBUG = obj_debug
BIN_DIR = bin

# Source files (exclude example files)
SOURCES = $(filter-out $(SRC_DIR)/udp_example.c, $(wildcard $(SRC_DIR)/*.c))
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJECTS_DEBUG = $(SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR_DEBUG)/%.o)

# Example source files
UDP_EXAMPLE_SRC = $(SRC_DIR)/udp_example.c
UDP_EXAMPLE_OBJ = $(OBJ_DIR)/udp_example.o
UDP_EXAMPLE_OBJ_DEBUG = $(OBJ_DIR_DEBUG)/udp_example.o

# Target executables
TARGET = $(BIN_DIR)/hardware_adapter
TARGET_DEBUG = $(BIN_DIR)/hardware_adapter_debug
UDP_EXAMPLE_TARGET = $(BIN_DIR)/udp_example
UDP_EXAMPLE_TARGET_DEBUG = $(BIN_DIR)/udp_example_debug

# Default target
all: $(TARGET)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Build target
$(TARGET): $(OBJECTS) | $(BIN_DIR)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Debug build target
debug: $(TARGET_DEBUG)

# Create debug directories
$(OBJ_DIR_DEBUG):
	mkdir -p $(OBJ_DIR_DEBUG)

# Build debug target
$(TARGET_DEBUG): $(OBJECTS_DEBUG) | $(BIN_DIR)
	$(CC) $(OBJECTS_DEBUG) -o $@ $(DEBUG_LDFLAGS)
	@echo "Debug build complete: $@"

# Compile debug source files
$(OBJ_DIR_DEBUG)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR_DEBUG)
	$(CC) $(DEBUG_CFLAGS) -c $< -o $@

# UDP example targets
udp-example: $(UDP_EXAMPLE_TARGET)

udp-example-debug: $(UDP_EXAMPLE_TARGET_DEBUG)

# Build UDP example (release)
$(UDP_EXAMPLE_TARGET): $(UDP_EXAMPLE_OBJ) | $(BIN_DIR)
	$(CC) $(UDP_EXAMPLE_OBJ) -o $@ -lm
	@echo "Build complete: $@"

# Build UDP example (debug)
$(UDP_EXAMPLE_TARGET_DEBUG): $(UDP_EXAMPLE_OBJ_DEBUG) | $(BIN_DIR)
	$(CC) $(UDP_EXAMPLE_OBJ_DEBUG) -o $@ -lm -g3
	@echo "Debug build complete: $@"

# Compile UDP example object files
$(UDP_EXAMPLE_OBJ): $(UDP_EXAMPLE_SRC) | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(UDP_EXAMPLE_OBJ_DEBUG): $(UDP_EXAMPLE_SRC) | $(OBJ_DIR_DEBUG)
	$(CC) $(DEBUG_CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(OBJ_DIR_DEBUG) $(BIN_DIR)
	@echo "Clean complete"

# Install dependencies (Ubuntu/Debian)
install-deps:
	sudo apt-get update
	sudo apt-get install -y libzmq3-dev build-essential
	@echo ""
	@echo "Note: MAVLink C library headers should be in ./mavlink/ directory"
	@echo "  The Makefile is configured to use local mavlink headers"

# Help target
help:
	@echo "Available targets:"
	@echo "  all              - Build the hardware adapter (optimized, default)"
	@echo "  debug            - Build the hardware adapter with debug symbols"
	@echo "  udp-example      - Build the UDP example (optimized)"
	@echo "  udp-example-debug - Build the UDP example with debug symbols"
	@echo "  clean            - Remove build artifacts"
	@echo "  install-deps     - Install required dependencies"
	@echo "  help             - Show this help message"

.PHONY: all debug udp-example udp-example-debug clean install-deps help

