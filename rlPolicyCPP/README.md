# RL Policy C++ Implementation

This directory contains C++ implementations for loading and running RL policies converted from PyTorch checkpoints.

## Files

- `RLPolicyJSON.h` / `RLPolicyJSON.cpp`: JSON-based policy implementation (no libtorch dependency)
- `main.cpp`: Simple program that loads a JSON policy and computes actions
- `test_comparison.cpp`: Test program that compares JSON-based vs libtorch implementations
- `Makefile`: Build configuration

## Prerequisites

### For `main` (JSON-based only):
- C++17 compiler (g++ or clang++)
- No external dependencies (uses standard library only)

### For `test_comparison` (comparison test):
- C++17 compiler
- LibTorch (PyTorch C++ API)
  - Download from: https://pytorch.org/get-started/locally/
  - Extract and set `TORCH_DIR` environment variable

## Building

### Build JSON-based program (no libtorch):
```bash
make rlPolicyCPP
```

### Build comparison test (requires libtorch):
```bash
export TORCH_DIR=/path/to/libtorch
make test_comparison
```

### Clean build artifacts:
```bash
make clean
```

## Usage

### Step 1: Convert .pth to JSON

First, convert your PyTorch checkpoint to JSON format:

```bash
cd /home/valentin/RL/src
python pth2json.py <path_to_checkpoint.pth> <output.json>
```

### Step 2: Run JSON-based policy

```bash
./bin/rlPolicyCPP --json=<jsonpath> --pth=<pthpath> [--nonlinearity=<relu|elu|tanh>] [--normalized=<true|false>]
```

Arguments:
- `--json=<jsonpath>`: Path to JSON file (converted from .pth) - **required**
- `--pth=<pthpath>`: Path to PyTorch checkpoint (.pth) - **required**
- `--nonlinearity=<type>`: Activation type: `relu`, `elu`, or `tanh` (default: `relu`) - **optional**
- `--normalized=<bool>`: Whether input is normalized: `true` or `false` (default: `false`) - **optional**

**Note**: Both `--json` and `--pth` are required. The program uses the JSON file for inference, but requires the PTH file path for validation/reference.

Examples:
```bash
./bin/rlPolicyCPP --json=policy.json --pth=checkpoint.pth
./bin/rlPolicyCPP --json=policy.json --pth=checkpoint.pth --nonlinearity=elu --normalized=false
./bin/rlPolicyCPP --json=policy.json --pth=checkpoint.pth --nonlinearity=tanh --normalized=true
```

### Step 3: Compare JSON vs libtorch (optional)

```bash
./bin/test_comparison <pth_file> <json_file> [nonlinearity] [normalized]
```

**Purpose**: This test program verifies that the manual JSON-based implementation produces the same inference results as the libtorch reference implementation.

**How it works**:
1. **Reference implementation**: Loads the `.pth` file using libtorch (the ground truth)
2. **Test implementation**: Loads the `.json` file using the manual implementation
3. **Comparison**: Runs forward pass with the same observation on both implementations
4. **Validation**: Compares outputs and reports differences to verify correctness

**Arguments**:
- `pth_file`: Path to original PyTorch checkpoint (used by libtorch reference)
- `json_file`: Path to JSON file (converted from .pth, used by manual implementation)
- `nonlinearity`: `relu`, `elu`, or `tanh` (default: `relu`) - must match training config
- `normalized`: `0` or `1` (default: `0`) - whether input is already normalized

**Example**:
```bash
./bin/test_comparison checkpoint.pth policy.json relu 0
```

**Output**: The program reports:
- Maximum absolute difference between outputs
- Maximum relative error
- Sample values for inspection
- Pass/fail status based on tolerance (default: 1e-4)

This test ensures that the manual neural network computation matches the libtorch reference, validating the correctness of the JSON-based implementation.

## Implementation Details

The JSON-based implementation (`RLPolicyJSON`) manually implements:
- Observation normalization (using running mean/var)
- Encoder MLP (Linear layers with activations)
- GRU recurrent layer
- Distribution head (outputs mean and logstd)

The forward pass follows the same structure as `rl_policyClean.py`:
1. Normalize observation: `(obs - mean) / sqrt(var + eps)`
2. Encoder forward: MLP with specified activation
3. GRU forward: Single-step GRU computation
4. Distribution forward: Linear layer producing [mean, logstd]

## Notes

- The JSON parser is a minimal implementation for the specific structure produced by `pth2json.py`
- For production use, consider using a proper JSON library (e.g., nlohmann/json)
- The GRU implementation uses a simplified single-step forward pass
- Hidden state is maintained internally between forward calls
- Use `reset_hidden_state()` to reset the RNN state
