# ============================================================================
# Hardware Adapter Makefile - Compilation Examples
# ============================================================================
#
# Available build targets and compilation variants:
#
# DEBUG BUILD (With debug symbols, DEFAULT - uses -O0 -g3):
#   make                    # Build both UDP and Serial adapters (debug with -g3, -O0)
#   make all               # Same as above - debug build (default)
#   make debug             # Same as above - explicit debug build
#   make debug-udp         # Build only UDP adapter with debug symbols
#   make debug-serial       # Build only Serial adapter with debug symbols
#
# RELEASE BUILD (Optimized - uses -O2):
#   make release           # Build both UDP and Serial adapters (optimized with -O2)
#   make release-udp       # Build only UDP adapter (optimized)
#   make release-serial     # Build only Serial adapter (optimized)
#
# INDIVIDUAL TARGETS:
#   make bin/zmq_commands_mavlink      # Build only ZMQ commands to MAVLink adapter (debug, default)
#   make bin/hardware_adapterSerial   # Build only Serial adapter (debug, default)
#
# EXAMPLE PROGRAMS:
#   make udp-example       # Build UDP example program (debug, default)
#   make udp-example-release # Build UDP example program (optimized)
#
# UTILITIES:
#   make clean             # Remove all build artifacts (obj/, obj_release/, bin/)
#   make install-deps      # Install required dependencies (libzmq3-dev, etc.)
#   make help              # Show available targets
#
# CUSTOM COMPILATION FLAGS:
#   To use custom flags, you can override CFLAGS:
#   make CFLAGS="-O0 -g -DVERBOSE" all        # Custom debug flags
#   make CFLAGS="-O3 -march=native" release   # Maximum optimization
#
# PROFILING BUILD:
#   make CFLAGS="-O2 -pg" LDFLAGS="-pthread -lzmq -lm -pg" release
#
# STATIC ANALYSIS:
#   make CC=clang CFLAGS="-Wall -Wextra -O0 -g -std=c11 -pthread -I. -Weverything" all
#
# ============================================================================

CC = gcc
# Include directories
# Add current directory to include path for local mavlink headers
INCLUDE_DIRS = -I.

# Debug flags (DEFAULT)
CFLAGS = -Wall -Wextra -g3 -O0 -std=c11 -pthread -DDEBUG $(INCLUDE_DIRS)
LDFLAGS = -pthread -lzmq -lm -g3

# Release flags (optimized)
RELEASE_CFLAGS = -Wall -Wextra -O2 -std=c11 -pthread $(INCLUDE_DIRS)
RELEASE_LDFLAGS = -pthread -lzmq -lm

# Directories
SRC_DIR = .
OBJ_DIR = obj              # Debug objects (default)
OBJ_DIR_RELEASE = obj_release  # Release objects
BIN_DIR = bin

# Source files for UDP adapter (exclude Serial adapter, Serial main, regular main, example files, and mavlink_to_ZMQ files)
# Include sine_test_module.c
UDP_SOURCES = $(filter-out hardware_adapterSerial.c mainSerial.c main.c udp_example.c mavlink_to_ZMQ.c main_mavlink_to_ZMQ.c, $(wildcard *.c))
UDP_OBJECTS = $(UDP_SOURCES:%.c=obj/%.o)              # Debug objects
UDP_OBJECTS_RELEASE = $(UDP_SOURCES:%.c=obj_release/%.o)  # Release objects

# Source files for Serial adapter (exclude UDP adapter, UDP main, regular main, example files, and mavlink_to_ZMQ files)
SERIAL_SOURCES = $(filter-out zmq_commands_mavlink.c main_zmq_commands_mavlink.c main.c udp_example.c mavlink_to_ZMQ.c main_mavlink_to_ZMQ.c, $(wildcard *.c))
SERIAL_OBJECTS = $(SERIAL_SOURCES:%.c=obj/%.o)              # Debug objects
SERIAL_OBJECTS_RELEASE = $(SERIAL_SOURCES:%.c=obj_release/%.o)  # Release objects

# Source files for MAVLink to ZMQ bridge
# Include common dependencies: zmq_wrapper.c, common.c, low_pass_filter.c
MAVLINK_TO_ZMQ_SOURCES = mavlink_to_ZMQ.c main_mavlink_to_ZMQ.c zmq_wrapper.c common.c low_pass_filter.c
MAVLINK_TO_ZMQ_OBJECTS = $(MAVLINK_TO_ZMQ_SOURCES:%.c=obj/%.o)              # Debug objects
MAVLINK_TO_ZMQ_OBJECTS_RELEASE = $(MAVLINK_TO_ZMQ_SOURCES:%.c=obj_release/%.o)  # Release objects

# Example source files
UDP_EXAMPLE_SRC = udp_example.c
UDP_EXAMPLE_OBJ = obj/udp_example.o
UDP_EXAMPLE_OBJ_RELEASE = obj_release/udp_example.o

# Target executables
UDP_TARGET = $(BIN_DIR)/zmq_commands_mavlink
UDP_TARGET_RELEASE = $(BIN_DIR)/zmq_commands_mavlink_release
SERIAL_TARGET = $(BIN_DIR)/hardware_adapterSerial
SERIAL_TARGET_RELEASE = $(BIN_DIR)/hardware_adapterSerial_release
MAVLINK_TO_ZMQ_TARGET = $(BIN_DIR)/mavlink_to_ZMQ
MAVLINK_TO_ZMQ_TARGET_RELEASE = $(BIN_DIR)/mavlink_to_ZMQ_release
UDP_EXAMPLE_TARGET = $(BIN_DIR)/udp_example
UDP_EXAMPLE_TARGET_RELEASE = $(BIN_DIR)/udp_example_release

# Default target (build all in debug mode)
all: debug

# Debug build (default)
debug: $(UDP_TARGET) $(SERIAL_TARGET) $(MAVLINK_TO_ZMQ_TARGET)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR_RELEASE):
	mkdir -p $(OBJ_DIR_RELEASE)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Build UDP target (debug, default)
$(UDP_TARGET): $(UDP_OBJECTS) | $(BIN_DIR)
	$(CC) $(UDP_OBJECTS) -o $@ $(LDFLAGS)
	@echo "Debug build complete: $@"

# Build Serial target (debug, default)
$(SERIAL_TARGET): $(SERIAL_OBJECTS) | $(BIN_DIR)
	$(CC) $(SERIAL_OBJECTS) -o $@ $(LDFLAGS)
	@echo "Debug build complete: $@"

# Build MAVLink to ZMQ target (debug, default)
$(MAVLINK_TO_ZMQ_TARGET): $(MAVLINK_TO_ZMQ_OBJECTS) | $(BIN_DIR)
	$(CC) $(MAVLINK_TO_ZMQ_OBJECTS) -o $@ $(LDFLAGS)
	@echo "Debug build complete: $@"

# Compile source files (debug, default) - pattern rule for all .c files
obj/%.o: %.c | obj
	$(CC) $(CFLAGS) -c $< -o $@

# Debug build targets (aliases)
debug-udp: $(UDP_TARGET)

debug-serial: $(SERIAL_TARGET)

debug-mavlink-to-zmq: $(MAVLINK_TO_ZMQ_TARGET)

mavlink-to-zmq: $(MAVLINK_TO_ZMQ_TARGET)

# Release build targets
release: $(UDP_TARGET_RELEASE) $(SERIAL_TARGET_RELEASE) $(MAVLINK_TO_ZMQ_TARGET_RELEASE)

release-udp: $(UDP_TARGET_RELEASE)

release-serial: $(SERIAL_TARGET_RELEASE)

release-mavlink-to-zmq: $(MAVLINK_TO_ZMQ_TARGET_RELEASE)

# Build release targets
$(UDP_TARGET_RELEASE): $(UDP_OBJECTS_RELEASE) | $(BIN_DIR)
	$(CC) $(UDP_OBJECTS_RELEASE) -o $@ $(RELEASE_LDFLAGS)
	@echo "Release build complete: $@"

$(SERIAL_TARGET_RELEASE): $(SERIAL_OBJECTS_RELEASE) | $(BIN_DIR)
	$(CC) $(SERIAL_OBJECTS_RELEASE) -o $@ $(RELEASE_LDFLAGS)
	@echo "Release build complete: $@"

$(MAVLINK_TO_ZMQ_TARGET_RELEASE): $(MAVLINK_TO_ZMQ_OBJECTS_RELEASE) | $(BIN_DIR)
	$(CC) $(MAVLINK_TO_ZMQ_OBJECTS_RELEASE) -o $@ $(RELEASE_LDFLAGS)
	@echo "Release build complete: $@"

# Compile release source files - pattern rule for all .c files
obj_release/%.o: %.c | obj_release
	$(CC) $(RELEASE_CFLAGS) -c $< -o $@

# UDP example targets
udp-example: $(UDP_EXAMPLE_TARGET)

udp-example-release: $(UDP_EXAMPLE_TARGET_RELEASE)

# Build UDP example (debug, default)
$(UDP_EXAMPLE_TARGET): $(UDP_EXAMPLE_OBJ) | $(BIN_DIR)
	$(CC) $(UDP_EXAMPLE_OBJ) -o $@ -lm -g3
	@echo "Debug build complete: $@"

# Build UDP example (release)
$(UDP_EXAMPLE_TARGET_RELEASE): $(UDP_EXAMPLE_OBJ_RELEASE) | $(BIN_DIR)
	$(CC) $(UDP_EXAMPLE_OBJ_RELEASE) -o $@ -lm
	@echo "Release build complete: $@"

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(OBJ_DIR_RELEASE) $(BIN_DIR)
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
	@echo "  all                    - Build UDP, Serial, and MAVLink to ZMQ adapters (debug, default)"
	@echo "  debug                  - Build all adapters with debug symbols (same as 'all')"
	@echo "  debug-udp              - Build UDP adapter with debug symbols"
	@echo "  debug-serial           - Build Serial adapter with debug symbols"
	@echo "  debug-mavlink-to-zmq   - Build MAVLink to ZMQ bridge with debug symbols"
	@echo "  mavlink-to-zmq         - Build MAVLink to ZMQ bridge (debug, default)"
	@echo "  release                - Build all adapters (optimized)"
	@echo "  release-udp            - Build UDP adapter (optimized)"
	@echo "  release-serial         - Build Serial adapter (optimized)"
	@echo "  release-mavlink-to-zmq - Build MAVLink to ZMQ bridge (optimized)"
	@echo "  udp-example            - Build the UDP example (debug, default)"
	@echo "  udp-example-release    - Build the UDP example (optimized)"
	@echo "  clean                  - Remove build artifacts"
	@echo "  install-deps           - Install required dependencies"
	@echo "  help                   - Show this help message"

.PHONY: all debug debug-udp debug-serial debug-mavlink-to-zmq mavlink-to-zmq release release-udp release-serial release-mavlink-to-zmq udp-example udp-example-release clean install-deps help

