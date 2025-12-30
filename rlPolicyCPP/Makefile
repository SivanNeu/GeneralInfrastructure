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

# Default: build JSON-only implementation (no libtorch required)
all: bin/rlPolicyCPP

# Create bin directory (order-only prerequisite)
bin:
	mkdir -p bin

# Build main program (JSON-only, no libtorch)
bin/rlPolicyCPP: bin/main.o bin/RLPolicyJSON.o | bin
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

bin/main.o: main.cpp RLPolicyJSON.h | bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

bin/RLPolicyJSON.o: RLPolicyJSON.cpp RLPolicyJSON.h | bin
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Note: test_comparison program removed (required libtorch)

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
	@echo "  make rlPolicyCPP       - Build policy program (JSON-only, no libtorch required)"
	@echo "  make clean             - Remove build artifacts"
	@echo ""
	@echo "Build modes (default: debug):"
	@echo "  make BUILD=debug rlPolicyCPP      - Build with debug symbols (-g -O0)"
	@echo "  make BUILD=release rlPolicyCPP   - Build optimized (-O3 -DNDEBUG)"
	@echo ""
	@echo "Current build mode: $(BUILD)"

.PHONY: all clean help main rlPolicyCPP
