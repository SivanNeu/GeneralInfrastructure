CXX = g++
INCLUDES = -I.
BIN_DIR = bin

# Build mode: debug (default) or release
BUILD ?= debug

# Compiler flags based on build mode
ifeq ($(BUILD),release)
    CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -DNDEBUG
else
    # Debug mode (default)
    CXXFLAGS = -std=c++17 -Wall -Wextra -g -O0 -DDEBUG
endif

# Paths - adjust these based on your system
TORCH_DIR ?= /path/to/libtorch
TORCH_INCLUDE = -I$(TORCH_DIR)/include -I$(TORCH_DIR)/include/torch/csrc/api/include
TORCH_LIBS = -L$(TORCH_DIR)/lib -ltorch -ltorch_cpu -lc10

# Default: build without libtorch (JSON-only)
all: bin/rlPolicyCPP

# Create bin directory (order-only prerequisite)
bin:
	mkdir -p bin

# Build main program (JSON-based, no libtorch required)
bin/rlPolicyCPP: bin/main.o bin/RLPolicyJSON.o | bin
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

bin/main.o: main.cpp RLPolicyJSON.h | bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

bin/RLPolicyJSON.o: RLPolicyJSON.cpp RLPolicyJSON.h | bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Build test comparison program (requires libtorch)
bin/test_comparison: bin/test_comparison.o bin/RLPolicyJSON.o | bin
	$(CXX) $(CXXFLAGS) -o $@ $^ $(TORCH_LIBS) -Wl,-rpath,$(TORCH_DIR)/lib

bin/test_comparison.o: test_comparison.cpp RLPolicyJSON.h | bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TORCH_INCLUDE) -c $< -o $@

# Convenience aliases
main: bin/rlPolicyCPP
rlPolicyCPP: bin/rlPolicyCPP
test_comparison: bin/test_comparison

# Clean build artifacts
clean:
	rm -f bin/*.o
	rm -f bin/rlPolicyCPP bin/test_comparison

# Help target
help:
	@echo "Available targets:"
	@echo "  make rlPolicyCPP       - Build JSON-based policy program (no libtorch)"
	@echo "  make test_comparison   - Build comparison test (requires libtorch)"
	@echo "  make clean             - Remove build artifacts"
	@echo ""
	@echo "Build modes (default: debug):"
	@echo "  make BUILD=debug rlPolicyCPP      - Build with debug symbols (-g -O0)"
	@echo "  make BUILD=release rlPolicyCPP   - Build optimized (-O3 -DNDEBUG)"
	@echo ""
	@echo "Current build mode: $(BUILD)"
	@echo ""
	@echo "To build test_comparison, set TORCH_DIR environment variable:"
	@echo "  export TORCH_DIR=/path/to/libtorch"
	@echo "  make test_comparison"

.PHONY: all clean help main rlPolicyCPP test_comparison
