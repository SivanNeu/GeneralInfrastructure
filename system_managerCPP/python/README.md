# Python Inference Testing

This directory contains Python scripts for testing and comparing C++ inference results with the original PyTorch model.

## Files

- `rl_policy_clean.py`: Self-contained RL policy implementation for inference
- `test_inference.py`: Test script to compare C++ and Python inference results

## Usage

### Test Inference Comparison

Compare C++ inference (from log files) with Python inference (from .pth or .json file):

```bash
python test_inference.py <log_file.csv> <pth_file.pth> [options]
python test_inference.py <log_file.csv> <json_file.json> [options]
```

**Arguments:**
- `log_file.csv`: Path to CSV log file created by Control.cpp
- `model_file`: Path to PyTorch checkpoint file (.pth) or JSON file (.json) created by pth2json.py

**JSON File Behavior:**
- If a JSON file is provided, the script will:
  1. Extract the `original_pth_file` path from the JSON metadata
  2. Try to load from the original .pth file if it exists
  3. Fall back to loading weights directly from JSON if .pth is not available
- This allows testing even when the original .pth file is not accessible

**Options:**
- `--output <file.csv>`: Output comparison CSV file (default: `<log_file>_comparison.csv`)
- `--nonlinearity <relu|elu|tanh>`: Activation function (default: `relu`, must match training config)
- `--device <cpu|cuda>`: Device to run inference on (default: `cpu`)

**Example:**
```bash
python test_inference.py ../logs/20240101_120000_control_logs_VelocityRL.csv \
    ../train_dir/rlcat2_quad/checkpoint_p0/best_000003172_3248128_reward_176.079.pth \
    --output comparison_results.csv \
    --nonlinearity relu
```

## Output Format

The output CSV file contains:
- `row_index`: Row index from original log
- `timestamp`: Timestamp from log
- `obs/0`, `obs/1`, ...: Observation values
- `cpp_action_mean/0`, `cpp_action_logstd/0`, ...: C++ actions (if logged)
- `py_action_mean/0`, `py_action_logstd/0`, ...: Python inference results
- `diff_mean/0`, `diff_logstd/0`, ...: Differences between C++ and Python (if C++ actions available)

## Note on C++ Action Logging

Currently, `Control.cpp` does not log action outputs (mean and logstd). To enable full comparison:

1. Modify `Control.cpp::log_control_data()` to accept and log action_logits
2. Update `VelocityRLController::getCommand()` to pass action_logits to the logger
3. The log file will then contain `cpp_action_mean/0`, `cpp_action_logstd/0`, etc.

Without C++ action logging, the script will still output Python inference results for verification.

## Dependencies

- Python 3.7+
- PyTorch
- NumPy

Install dependencies:
```bash
pip install torch numpy
```
