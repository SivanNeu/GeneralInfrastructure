#!/usr/bin/env python3
"""
Test function to compare C++ and Python inference results.

Reads log files created by Control.cpp, runs Python inference using the policy file
specified in the log metadata, and outputs a comparison CSV with observations,
C++ actions, Python actions, and differences.

For RL log files, the policy file path is automatically extracted from the log metadata.
The script will also try to load the original .pth file from the JSON if available.

Usage:
    python test_inference.py --log=log_file_rl_vfvr.csv
    python test_inference.py --log=log_file_rl_yaw.csv [--output=comparison.csv] [--device=cpu]
"""
import argparse
import csv
import os
import sys
import json
import numpy as np
import torch

from rl_policy_clean import RLPolicyClean


def format_value(value, width=12):
    """Format a value to a fixed width string.
    
    Args:
        value: Value to format (can be number, string, or None)
        width: Target width in characters (default: 12)
    
    Returns:
        Formatted string with specified width
    """
    if value is None or value == '':
        return ' ' * width
    if isinstance(value, (int, float, np.number)):
        # Format numbers with appropriate precision
        if isinstance(value, float) or isinstance(value, np.floating):
            # Use scientific notation for very small/large numbers, otherwise fixed decimal
            if abs(value) < 1e-3 and value != 0:
                return f"{value:12.6e}"
            elif abs(value) > 1e6:
                return f"{value:12.6e}"
            else:
                return f"{value:12.6f}"
        else:
            return f"{value:12d}"
    # For strings, truncate or pad as needed
    s = str(value)
    if len(s) > width:
        return s[:width]
    return f"{s:<{width}}"


def read_log_file(log_file_path: str) -> list[dict]:
    """Read CSV log file created by Control.cpp Logger.
    
    Args:
        log_file_path: Path to the CSV log file
        
    Returns:
        List of dictionaries, one per row, with all column values
    """
    rows = []
    with open(log_file_path, 'r') as f:
        # Skip comment lines (starting with #)
        lines = f.readlines()
        data_lines = [line for line in lines if not line.strip().startswith('#')]
        
        reader = csv.DictReader(data_lines)
        for row in reader:
            rows.append(row)
    return rows


def read_rl_log_metadata(log_file_path: str) -> str | None:
    """Read policy file path from RL log file metadata.
    
    Args:
        log_file_path: Path to the CSV RL log file
        
    Returns:
        Policy file path if found, None otherwise
    """
    try:
        with open(log_file_path, 'r') as f:
            first_line = f.readline().strip()
            if first_line.startswith('# policy_file:'):
                # Extract the path after '# policy_file: '
                policy_path = first_line.replace('# policy_file:', '').strip()
                return policy_path if policy_path else None
    except Exception:
        pass
    return None


def get_pth_file_from_json(json_file_path: str) -> str | None:
    """Extract .pth file path from JSON file.
    
    Args:
        json_file_path: Path to JSON file
        
    Returns:
        Path to .pth file if found in JSON, None otherwise
    """
    try:
        with open(json_file_path, 'r') as f:
            json_data = json.load(f)
            original_pth_path = json_data.get('original_pth_file')
            if original_pth_path:
                # Try to resolve relative paths
                if not os.path.isabs(original_pth_path):
                    json_dir = os.path.dirname(json_file_path)
                    original_pth_path = os.path.join(json_dir, original_pth_path)
                    original_pth_path = os.path.abspath(original_pth_path)
                return original_pth_path if os.path.exists(original_pth_path) else None
    except Exception as e:
        print(f"Warning: Failed to read .pth path from JSON: {e}")
    return None


def is_rl_log_file(log_file_path: str) -> bool:
    """Check if the log file is an RL log file (contains RL-specific columns or metadata).
    
    Args:
        log_file_path: Path to the CSV log file
        
    Returns:
        True if it's an RL log file, False otherwise
    """
    try:
        # Check for metadata first
        policy_path = read_rl_log_metadata(log_file_path)
        if policy_path:
            return True
        
        # Check for RL-specific columns
        with open(log_file_path, 'r') as f:
            reader = csv.DictReader(f)
            first_row = next(reader, None)
            if first_row is None:
                return False
            # Check for RL-specific columns (new format: obs/, mean/, logstd/, hxs/)
            return 'obs/0' in first_row or 'mean/0' in first_row or 'hxs/0' in first_row
    except Exception:
        return False


def extract_observations(row: dict) -> np.ndarray | None:
    """Extract observation vector from a log row.
    
    Args:
        row: Dictionary containing log row data
        
    Returns:
        NumPy array of observations, or None if not found
    """
    obs_values = []
    i = 0
    while f'obs/{i}' in row:
        try:
            val = float(row[f'obs/{i}'])
            obs_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    if len(obs_values) == 0:
        return None
    
    return np.array(obs_values, dtype=np.float32)


def extract_rl_observations(row: dict, is_vfvr: bool = True) -> np.ndarray | None:
    """Extract observation vector from an RL log row.
    
    Args:
        row: Dictionary containing log row data
        is_vfvr: If True, extract obsXY (4 values), else extract obsHeading (2 values)
        
    Returns:
        NumPy array of observations, or None if not found
    """
    obs_values = []
    
    # Try new format first (obs/0, obs/1, ...)
    i = 0
    while f'obs/{i}' in row:
        try:
            val = float(row[f'obs/{i}'])
            obs_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    # Fall back to old format if new format not found
    if len(obs_values) == 0:
        if is_vfvr:
            # Try obsXY format
            i = 0
            while f'obsXY/{i}' in row:
                try:
                    val = float(row[f'obsXY/{i}'])
                    obs_values.append(val)
                    i += 1
                except (ValueError, KeyError):
                    break
        else:
            # Try obsHeading format
            i = 0
            while f'obsHeading/{i}' in row:
                try:
                    val = float(row[f'obsHeading/{i}'])
                    obs_values.append(val)
                    i += 1
                except (ValueError, KeyError):
                    break
    
    return np.array(obs_values, dtype=np.float32) if len(obs_values) > 0 else None


def extract_rl_hidden_states(row: dict) -> np.ndarray | None:
    """Extract hidden states from an RL log row.
    
    Args:
        row: Dictionary containing log row data
        
    Returns:
        NumPy array of hidden states, or None if not found
    """
    hxs_values = []
    
    # Try new format with zero-padded indices (z_hxs/000, z_hxs/001, ...)
    # First, find all z_hxs/ keys and sort them numerically
    z_hxs_keys = [key for key in row.keys() if key.startswith('z_hxs/')]
    if z_hxs_keys:
        # Extract indices and sort numerically
        def extract_index(key):
            try:
                # Remove 'z_hxs/' prefix and parse as int
                idx_str = key.replace('z_hxs/', '')
                return int(idx_str)
            except (ValueError, AttributeError):
                return -1
        
        # Sort by index
        z_hxs_keys_sorted = sorted(z_hxs_keys, key=extract_index)
        
        for key in z_hxs_keys_sorted:
            try:
                val_str = row[key]
                # Skip None or empty values
                if val_str is None or val_str == '':
                    continue
                val = float(val_str)
                hxs_values.append(val)
            except (ValueError, KeyError, TypeError):
                continue
        
        if len(hxs_values) > 0:
            return np.array(hxs_values, dtype=np.float32)
    
    # Fall back to non-zero-padded format (z_hxs/0, z_hxs/1, ...)
    i = 0
    while f'z_hxs/{i}' in row:
        try:
            val_str = row[f'z_hxs/{i}']
            # Skip None or empty values
            if val_str is None or val_str == '':
                break
            val = float(val_str)
            hxs_values.append(val)
            i += 1
        except (ValueError, KeyError, TypeError):
            break
    
    if len(hxs_values) > 0:
        return np.array(hxs_values, dtype=np.float32)
    
    # Fall back to hxs/ format (without z_ prefix) for backward compatibility
    if len(hxs_values) == 0:
        i = 0
        while f'hxs/{i}' in row:
            try:
                val_str = row[f'hxs/{i}']
                # Skip None or empty values
                if val_str is None or val_str == '':
                    break
                val = float(val_str)
                hxs_values.append(val)
                i += 1
            except (ValueError, KeyError, TypeError):
                break
    
    # Fall back to old format if new format not found
    if len(hxs_values) == 0:
        # Try hxs_vfvr or hxs_yaw format (for backward compatibility)
        i = 0
        while f'hxs_vfvr/{i}' in row:
            try:
                val_str = row[f'hxs_vfvr/{i}']
                # Skip None or empty values
                if val_str is None or val_str == '':
                    break
                val = float(val_str)
                hxs_values.append(val)
                i += 1
            except (ValueError, KeyError, TypeError):
                break
        
        if len(hxs_values) == 0:
            i = 0
            while f'hxs_yaw/{i}' in row:
                try:
                    val_str = row[f'hxs_yaw/{i}']
                    # Skip None or empty values
                    if val_str is None or val_str == '':
                        break
                    val = float(val_str)
                    hxs_values.append(val)
                    i += 1
                except (ValueError, KeyError, TypeError):
                    break
    
    return np.array(hxs_values, dtype=np.float32) if len(hxs_values) > 0 else None


def extract_rl_cpp_actions_single(row: dict, is_vfvr: bool = True) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """Extract C++ actions and distributions from an RL log row (single policy format).
    
    Args:
        row: Dictionary containing log row data
        is_vfvr: If True, extract vfvr data (action/x, action/y, mean, logstd), else extract yaw data (action/yaw, mean, logstd)
        
    Returns:
        Tuple of (action, mean, logstd)
        For vfvr: action is [action/x, action/y], mean and logstd are arrays
        For yaw: action is [action/yaw], mean and logstd are arrays
    """
    # Extract actual actions
    if is_vfvr:
        action_x = float(row.get('action/x', 0.0)) if 'action/x' in row else None
        action_y = float(row.get('action/y', 0.0)) if 'action/y' in row else None
        action = None
        if action_x is not None and action_y is not None:
            action = np.array([action_x, action_y], dtype=np.float32)
    else:
        action_yaw = float(row.get('action/yaw', 0.0)) if 'action/yaw' in row else None
        action = np.array([action_yaw], dtype=np.float32) if action_yaw is not None else None
    
    # Extract mean and logstd (new format: mean/0, logstd/0)
    mean_values = []
    logstd_values = []
    i = 0
    while f'mean/{i}' in row:
        try:
            val = float(row[f'mean/{i}'])
            mean_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    i = 0
    while f'logstd/{i}' in row:
        try:
            val = float(row[f'logstd/{i}'])
            logstd_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    mean = np.array(mean_values, dtype=np.float32) if len(mean_values) > 0 else None
    logstd = np.array(logstd_values, dtype=np.float32) if len(logstd_values) > 0 else None
    
    return (action, mean, logstd)


def extract_rl_cpp_actions(row: dict) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """Extract C++ actions and distributions from an RL log row.
    
    Args:
        row: Dictionary containing log row data
        
    Returns:
        Tuple of (action_xy, action_yaw, mean_vfvr, logstd_vfvr, mean_yaw, logstd_yaw)
        action_xy is [action/x, action/y], action_yaw is [action/yaw]
    """
    # Extract actual actions
    action_x = float(row.get('action/x', 0.0)) if 'action/x' in row else None
    action_y = float(row.get('action/y', 0.0)) if 'action/y' in row else None
    action_yaw = float(row.get('action/yaw', 0.0)) if 'action/yaw' in row else None
    
    action_xy = None
    if action_x is not None and action_y is not None:
        action_xy = np.array([action_x, action_y], dtype=np.float32)
    action_yaw_arr = np.array([action_yaw], dtype=np.float32) if action_yaw is not None else None
    
    # Extract mean and logstd for vfvr
    mean_vfvr_values = []
    logstd_vfvr_values = []
    i = 0
    while f'mean_vfvr/{i}' in row:
        try:
            val = float(row[f'mean_vfvr/{i}'])
            mean_vfvr_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    i = 0
    while f'logstd_vfvr/{i}' in row:
        try:
            val = float(row[f'logstd_vfvr/{i}'])
            logstd_vfvr_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    mean_vfvr = np.array(mean_vfvr_values, dtype=np.float32) if len(mean_vfvr_values) > 0 else None
    logstd_vfvr = np.array(logstd_vfvr_values, dtype=np.float32) if len(logstd_vfvr_values) > 0 else None
    
    # Extract mean and logstd for yaw
    mean_yaw_values = []
    logstd_yaw_values = []
    i = 0
    while f'mean_yaw/{i}' in row:
        try:
            val = float(row[f'mean_yaw/{i}'])
            mean_yaw_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    i = 0
    while f'logstd_yaw/{i}' in row:
        try:
            val = float(row[f'logstd_yaw/{i}'])
            logstd_yaw_values.append(val)
            i += 1
        except (ValueError, KeyError):
            break
    
    mean_yaw = np.array(mean_yaw_values, dtype=np.float32) if len(mean_yaw_values) > 0 else None
    logstd_yaw = np.array(logstd_yaw_values, dtype=np.float32) if len(logstd_yaw_values) > 0 else None
    
    return (action_xy, action_yaw_arr, mean_vfvr, logstd_vfvr, mean_yaw, logstd_yaw)


def load_hxs_from_file(hxs_file_path: str, policy_type: str = 'auto') -> tuple[torch.Tensor | None, torch.Tensor | None]:
    """Load hidden state data from JSON file.
    
    Args:
        hxs_file_path: Path to the hxs JSON file
        policy_type: Which policy to use ('vfvr', 'yaw', or 'auto' to infer from file)
        
    Returns:
        Tuple of (hxs_vfvr, hxs_yaw) tensors. One will be None if not found or not requested.
        If policy_type is 'auto', returns the first non-empty hxs found.
    """
    if not os.path.exists(hxs_file_path):
        print(f"Warning: hxs file not found: {hxs_file_path}")
        return (None, None)
    
    try:
        with open(hxs_file_path, 'r') as f:
            hxs_data = json.load(f)
        
        hxs_vfvr = None
        hxs_yaw = None
        
        if 'hxs_vfvr' in hxs_data:
            hxs_vfvr_list = hxs_data['hxs_vfvr']
            if len(hxs_vfvr_list) > 0:
                hxs_vfvr = torch.tensor(hxs_vfvr_list, dtype=torch.float32)
        
        if 'hxs_yaw' in hxs_data:
            hxs_yaw_list = hxs_data['hxs_yaw']
            if len(hxs_yaw_list) > 0:
                hxs_yaw = torch.tensor(hxs_yaw_list, dtype=torch.float32)
        
        if policy_type == 'auto':
            # Return the first non-empty hxs found
            if hxs_vfvr is not None:
                return (hxs_vfvr, None)
            elif hxs_yaw is not None:
                return (None, hxs_yaw)
            else:
                return (None, None)
        elif policy_type == 'vfvr':
            return (hxs_vfvr, None)
        elif policy_type == 'yaw':
            return (None, hxs_yaw)
        elif policy_type == 'both':
            return (hxs_vfvr, hxs_yaw)
        else:
            return (hxs_vfvr, hxs_yaw)
            
    except Exception as e:
        print(f"Warning: Failed to load hxs file {hxs_file_path}: {e}")
        return (None, None)


def extract_cpp_actions(row: dict, action_dim: int) -> tuple[np.ndarray, np.ndarray] | None:
    """Extract C++ actions from log row if available.
    
    Note: C++ actions are not currently logged by Control.cpp. This function
    is a placeholder for when action logging is added.
    
    Args:
        row: Dictionary containing log row data
        action_dim: Expected action dimension (for mean and logstd)
        
    Returns:
        Tuple of (mean, logstd) arrays, or None if not found
    """
    # Check if C++ actions are logged (they're not currently)
    # This would look for keys like "cpp_action_mean/0", "cpp_action_logstd/0", etc.
    mean_values = []
    logstd_values = []
    
    for i in range(action_dim):
        mean_key = f'cpp_action_mean/{i}'
        logstd_key = f'cpp_action_logstd/{i}'
        if mean_key in row and logstd_key in row:
            try:
                mean_values.append(float(row[mean_key]))
                logstd_values.append(float(row[logstd_key]))
            except (ValueError, KeyError):
                return None
        else:
            return None
    
    if len(mean_values) == action_dim and len(logstd_values) == action_dim:
        return (np.array(mean_values), np.array(logstd_values))
    
    return None


def load_policy_from_file(
    model_file_path: str,
    nonlinearity: str = 'relu',
    device: str = 'cpu'
) -> RLPolicyClean:
    """Load policy from either .pth or .json file.
    
    If JSON file is provided:
    1. Extract original_pth_file path from JSON
    2. Try to load from .pth if available
    3. Fall back to JSON if .pth is not available
    
    Args:
        model_file_path: Path to .pth or .json file
        nonlinearity: Activation function ('relu', 'elu', 'tanh')
        device: Device to run inference on ('cpu' or 'cuda')
        
    Returns:
        Loaded RLPolicyClean instance
    """
    model_file_path = os.path.abspath(model_file_path)
    
    if model_file_path.endswith('.json'):
        # Load from JSON file
        print(f"Loading JSON file: {model_file_path}")
        
        with open(model_file_path, 'r') as f:
            json_data = json.load(f)
        
        # Extract original .pth file path if available
        original_pth_path = json_data.get('original_pth_file')
        if original_pth_path:
            # Try to resolve relative paths
            if not os.path.isabs(original_pth_path):
                # If relative, try relative to JSON file location
                json_dir = os.path.dirname(model_file_path)
                original_pth_path = os.path.join(json_dir, original_pth_path)
                original_pth_path = os.path.abspath(original_pth_path)
            
            print(f"Found original .pth file path in JSON: {original_pth_path}")
            
            # Try to load from .pth file if it exists
            if os.path.exists(original_pth_path):
                print(f"Loading policy from original .pth file: {original_pth_path}")
                try:
                    policy = RLPolicyClean.load_from_checkpoint(
                        original_pth_path,
                        device=device,
                        nonlinearity=nonlinearity,
                        jit_encoder=False
                    ).eval()
                    print("Successfully loaded from .pth file")
                    return policy
                except Exception as e:
                    print(f"Warning: Failed to load from .pth file: {e}")
                    print("Falling back to JSON-based loading...")
            else:
                print(f"Original .pth file not found at: {original_pth_path}")
                print("Using JSON data for inference...")
        
        # Load from JSON (either .pth not found or not specified)
        print(f"Loading policy from JSON file: {model_file_path}")
        policy = RLPolicyClean.load_from_json(
            model_file_path,
            device=device,
            nonlinearity=nonlinearity
        ).eval()
        print("Successfully loaded from JSON file")
        return policy
    
    elif model_file_path.endswith('.pth'):
        # Load from .pth file directly
        print(f"Loading policy from .pth file: {model_file_path}")
        policy = RLPolicyClean.load_from_checkpoint(
            model_file_path,
            device=device,
            nonlinearity=nonlinearity,
            jit_encoder=False
        ).eval()
        return policy
    
    else:
        raise ValueError(f"Unsupported file format. Expected .pth or .json, got: {model_file_path}")


def run_inference_test_rl_single(
    log_file_path: str,
    output_file_path: str = None,
    device: str = 'cpu'
):
    """Main test function to compare C++ and Python inference for a single RL log file (vfvr or yaw).
    
    Args:
        log_file_path: Path to CSV RL log file from Control.cpp (either _rl_vfvr.csv or _rl_yaw.csv)
        output_file_path: Path to output comparison CSV (default: <log_file>_comparison.csv)
        device: Device to run inference on ('cpu' or 'cuda')
    """
    # Auto-detect policy file from log metadata
    policy_file_path = read_rl_log_metadata(log_file_path)
    if not policy_file_path:
        print("Error: Could not find policy file path in log metadata.")
        print("Expected first line: # policy_file: /path/to/policy.json")
        sys.exit(1)
    
    print(f"Auto-detected policy file from log metadata: {policy_file_path}")
    
    if not os.path.exists(policy_file_path):
        print(f"Error: Policy file not found: {policy_file_path}")
        sys.exit(1)
    
    # Determine if this is a vfvr or yaw log file
    is_vfvr_log = '_rl_vfvr' in log_file_path or 'vfvr' in log_file_path.lower()
    is_yaw_log = '_rl_yaw' in log_file_path or 'yaw' in log_file_path.lower()
    
    if not (is_vfvr_log or is_yaw_log):
        print("Warning: Could not determine if this is a vfvr or yaw log file.")
        print("Assuming based on file name...")
        is_vfvr_log = 'vfvr' in log_file_path.lower()
        is_yaw_log = not is_vfvr_log
    
    policy_type = 'vfvr' if is_vfvr_log else 'yaw'
    print(f"Processing {policy_type} log file")
    
    # Load policy (load_policy_from_file will automatically extract .pth from JSON if available)
    policy = load_policy_from_file(policy_file_path, 'relu', device)
    
    # Determine action dimension
    action_dim = policy.dist_linear.out_features // 2
    print(f"Action dimension: {action_dim}")
    
    print(f"Reading RL log file: {log_file_path}")
    log_rows = read_log_file(log_file_path)
    print(f"Found {len(log_rows)} log entries")
    
    if output_file_path is None:
        base_name = os.path.splitext(log_file_path)[0]
        output_file_path = f"{base_name}_comparison.csv"
    
    print(f"Writing comparison results to: {output_file_path}")
    
    # Prepare output CSV
    with open(output_file_path, 'w', newline='') as f:
        # Build header
        fieldnames = ['row_index', 'timestamp']
        
        # Observation columns
        max_obs_dim = 0
        for row in log_rows:
            obs = extract_rl_observations(row, is_vfvr=is_vfvr_log)
            if obs is not None:
                max_obs_dim = max(max_obs_dim, len(obs))
        
        for i in range(max_obs_dim):
            fieldnames.append(f'obs/{i}')
        
        # C++ distribution columns
        fieldnames.extend([f'cpp_mean/{i}' for i in range(action_dim)])
        fieldnames.extend([f'cpp_logstd/{i}' for i in range(action_dim)])
        
        # Python distribution columns
        fieldnames.extend([f'py_mean/{i}' for i in range(action_dim)])
        fieldnames.extend([f'py_logstd/{i}' for i in range(action_dim)])
        
        # Difference columns (only mean and logstd, not actions)
        fieldnames.extend([f'diff_mean/{i}' for i in range(action_dim)])
        fieldnames.extend([f'diff_logstd/{i}' for i in range(action_dim)])
        
        # Hidden state columns - determine max dimension
        max_hxs_dim = 0
        for row in log_rows:
            hxs = extract_rl_hidden_states(row)
            if hxs is not None:
                max_hxs_dim = max(max_hxs_dim, len(hxs))
        
        # Add C++ hxs columns (from log file)
        fieldnames.extend([f'cpp_hxs/{i}' for i in range(max_hxs_dim)])
        # Add Python hxs columns (from inference)
        fieldnames.extend([f'py_hxs/{i}' for i in range(max_hxs_dim)])
        # Add difference columns
        fieldnames.extend([f'diff_hxs/{i}' for i in range(max_hxs_dim)])
        
        # Write formatted header
        header_line = ' '.join([format_value(fieldname, 12) for fieldname in fieldnames])
        f.write(header_line + '\n')
        
        # Initialize hidden state to zeros for the first row
        # The hxs in row N is the state AFTER processing row N, so it should be used as input for row N+1
        policy.reset_hidden_state(batch_size=1, device=torch.device(device))
        hxs_prev = None  # Will store the hidden state from the previous row
        
        # Process each log row
        for row_idx, row in enumerate(log_rows):
            # Extract observations
            obs = extract_rl_observations(row, is_vfvr=is_vfvr_log)
            if obs is None:
                print(f"Warning: Row {row_idx} has missing observations, skipping")
                continue
            
            # Extract C++ actions and distributions
            cpp_action, cpp_mean, cpp_logstd = extract_rl_cpp_actions_single(row, is_vfvr=is_vfvr_log)
            
            # Extract hidden states from log (these are the states AFTER the inference that produced the logged actions)
            hxs_log = extract_rl_hidden_states(row)
            
            # Set hidden state for this inference
            # For row 0: use zeros (first inference starts with zero hidden state)
            # For row N+1: use hxs from row N (the previous row's hxs_after, stored in hxs_prev)
            if row_idx > 0 and hxs_prev is not None and len(hxs_prev) > 0:
                # Use the hidden state from the previous row (which is the state AFTER that row's inference)
                hxs_tensor = torch.from_numpy(hxs_prev).unsqueeze(0)
                if hxs_tensor.dim() == 1:
                    hxs_tensor = hxs_tensor.unsqueeze(0)
                policy.set_hidden_state(hxs_tensor)
            # For row 0, we already reset to zeros above, so no need to do anything
            
            # Get timestamp
            timestamp = row.get('timestamp', str(row_idx))
            
            # Prepare output row
            output_row = {
                'row_index': row_idx,
                'timestamp': timestamp
            }
            
            # Add observations
            for i in range(max_obs_dim):
                if i < len(obs):
                    output_row[f'obs/{i}'] = obs[i]
                else:
                    output_row[f'obs/{i}'] = ''
            
            # Add C++ distributions
            for i in range(action_dim):
                output_row[f'cpp_mean/{i}'] = cpp_mean[i] if cpp_mean is not None and i < len(cpp_mean) else ''
                output_row[f'cpp_logstd/{i}'] = cpp_logstd[i] if cpp_logstd is not None and i < len(cpp_logstd) else ''
            
            # Run Python inference
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)  # [1, obs_dim]
                action_logits, h_next = policy(obs_tensor, normalized=False)
                action_logits = action_logits.squeeze(0)  # [2*action_dim]
                py_mean = action_logits[:action_dim].numpy()
                py_logstd = action_logits[action_dim:].numpy()
                # Extract Python hidden state (h_next is [batch, hidden_size] from policy)
                # Convert to numpy array, handling different tensor shapes
                if h_next is not None:
                    if isinstance(h_next, torch.Tensor):
                        # h_next is [batch, hidden_size], squeeze batch dimension if batch_size=1
                        if h_next.dim() > 1 and h_next.shape[0] == 1:
                            py_hxs = h_next.squeeze(0).cpu().numpy()
                        else:
                            py_hxs = h_next.cpu().numpy()
                    else:
                        py_hxs = np.array(h_next)
                else:
                    py_hxs = None
            
            # Add Python distributions
            for i in range(action_dim):
                output_row[f'py_mean/{i}'] = py_mean[i]
                output_row[f'py_logstd/{i}'] = py_logstd[i]
            
            # Calculate differences
            if cpp_mean is not None:
                diff_mean = py_mean - cpp_mean
                for i in range(action_dim):
                    output_row[f'diff_mean/{i}'] = diff_mean[i] if i < len(diff_mean) else ''
            else:
                for i in range(action_dim):
                    output_row[f'diff_mean/{i}'] = ''
            
            if cpp_logstd is not None:
                diff_logstd = py_logstd - cpp_logstd
                for i in range(action_dim):
                    output_row[f'diff_logstd/{i}'] = diff_logstd[i] if i < len(diff_logstd) else ''
            else:
                for i in range(action_dim):
                    output_row[f'diff_logstd/{i}'] = ''
            
            # Add C++ hidden states (from log file) - these are the states AFTER the inference that produced the logged actions
            if hxs_log is not None:
                for i in range(max_hxs_dim):
                    if i < len(hxs_log):
                        output_row[f'cpp_hxs/{i}'] = hxs_log[i]
                    else:
                        output_row[f'cpp_hxs/{i}'] = ''
            else:
                for i in range(max_hxs_dim):
                    output_row[f'cpp_hxs/{i}'] = ''
            
            # Add Python hidden states (from inference) - these are the states AFTER this inference
            if py_hxs is not None:
                for i in range(max_hxs_dim):
                    if i < len(py_hxs):
                        output_row[f'py_hxs/{i}'] = py_hxs[i]
                    else:
                        output_row[f'py_hxs/{i}'] = ''
            else:
                for i in range(max_hxs_dim):
                    output_row[f'py_hxs/{i}'] = ''
            
            # Calculate hxs differences
            if hxs_log is not None and py_hxs is not None:
                min_len = min(len(hxs_log), len(py_hxs))
                diff_hxs = py_hxs[:min_len] - hxs_log[:min_len]
                for i in range(max_hxs_dim):
                    if i < len(diff_hxs):
                        output_row[f'diff_hxs/{i}'] = diff_hxs[i]
                    else:
                        output_row[f'diff_hxs/{i}'] = ''
            else:
                for i in range(max_hxs_dim):
                    output_row[f'diff_hxs/{i}'] = ''
            
            # Write formatted row
            formatted_row = ' '.join([format_value(output_row.get(fieldname, ''), 12) for fieldname in fieldnames])
            f.write(formatted_row + '\n')
            
            # Store this row's hidden state for use in the next row
            # Use Python hxs (h_next) for next row, as that's what was actually used in the inference
            # If py_hxs is available, use it; otherwise fall back to hxs_log
            hxs_prev = py_hxs if py_hxs is not None else hxs_log
            
            # Progress indicator
            if (row_idx + 1) % 100 == 0:
                print(f"Processed {row_idx + 1}/{len(log_rows)} rows...")
    
    print(f"\nComparison complete! Results written to: {output_file_path}")
    
    # Print summary statistics
    print("\nCalculating summary statistics...")
    with open(output_file_path, 'r') as f:
        # Read header line (first line)
        header_line = f.readline().strip()
        fieldnames = [field.strip() for field in header_line.split()]
        reader = csv.DictReader(f, fieldnames=fieldnames, delimiter=' ', skipinitialspace=True)
        diff_means = []
        diff_logstds = []
        diff_hxs_all = []
        
        for row in reader:
            for i in range(action_dim):
                key = f'diff_mean/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_means.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
                key = f'diff_logstd/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_logstds.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
            
            # Collect hxs differences
            for i in range(max_hxs_dim):
                key = f'diff_hxs/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_hxs_all.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
    
    if diff_means:
        print(f"\nMean difference statistics:")
        print(f"  Mean: {np.mean(diff_means):.6e}")
        print(f"  Std:  {np.std(diff_means):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_means)):.6e}")
    
    if diff_logstds:
        print(f"Logstd difference statistics:")
        print(f"  Mean: {np.mean(diff_logstds):.6e}")
        print(f"  Std:  {np.std(diff_logstds):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_logstds)):.6e}")
    
    if diff_hxs_all:
        print(f"Hidden state difference statistics:")
        print(f"  Mean: {np.mean(diff_hxs_all):.6e}")
        print(f"  Std:  {np.std(diff_hxs_all):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_hxs_all)):.6e}")


def run_inference_test_rl(
    log_file_path: str,
    model_file_path_vfvr: str,
    model_file_path_yaw: str,
    output_file_path: str = None,
    nonlinearity: str = 'relu',
    device: str = 'cpu'
):
    """Main test function to compare C++ and Python inference for RL log files.
    
    Args:
        log_file_path: Path to CSV RL log file from Control.cpp
        model_file_path_vfvr: Path to PyTorch .pth checkpoint file or JSON file for vfvr policy
        model_file_path_yaw: Path to PyTorch .pth checkpoint file or JSON file for yaw policy
        output_file_path: Path to output comparison CSV (default: <log_file>_comparison.csv)
        nonlinearity: Activation function ('relu', 'elu', 'tanh')
        device: Device to run inference on ('cpu' or 'cuda')
    """
    # Load both policies
    print("Loading vfvr policy...")
    policy_vfvr = load_policy_from_file(model_file_path_vfvr, nonlinearity, device)
    print("Loading yaw policy...")
    policy_yaw = load_policy_from_file(model_file_path_yaw, nonlinearity, device)
    
    # Determine action dimensions
    action_dim_vfvr = policy_vfvr.dist_linear.out_features // 2
    action_dim_yaw = policy_yaw.dist_linear.out_features // 2
    print(f"Vfvr action dimension: {action_dim_vfvr}")
    print(f"Yaw action dimension: {action_dim_yaw}")
    
    print(f"Reading RL log file: {log_file_path}")
    log_rows = read_log_file(log_file_path)
    print(f"Found {len(log_rows)} log entries")
    
    if output_file_path is None:
        base_name = os.path.splitext(log_file_path)[0]
        output_file_path = f"{base_name}_comparison.csv"
    
    print(f"Writing comparison results to: {output_file_path}")
    
    # Prepare output CSV
    with open(output_file_path, 'w', newline='') as f:
        # Build header
        fieldnames = ['row_index', 'timestamp']
        
        # Observation columns
        fieldnames.extend([f'obsXY/{i}' for i in range(4)])
        fieldnames.extend([f'obsHeading/{i}' for i in range(2)])
        
        # C++ action columns
        fieldnames.extend(['cpp_action/x', 'cpp_action/y', 'cpp_action/yaw'])
        fieldnames.extend([f'cpp_mean_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'cpp_logstd_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'cpp_mean_yaw/{i}' for i in range(action_dim_yaw)])
        fieldnames.extend([f'cpp_logstd_yaw/{i}' for i in range(action_dim_yaw)])
        
        # Python action columns
        fieldnames.extend([f'py_mean_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'py_logstd_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'py_mean_yaw/{i}' for i in range(action_dim_yaw)])
        fieldnames.extend([f'py_logstd_yaw/{i}' for i in range(action_dim_yaw)])
        
        # Difference columns (only mean and logstd, not actions)
        fieldnames.extend([f'diff_mean_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'diff_logstd_vfvr/{i}' for i in range(action_dim_vfvr)])
        fieldnames.extend([f'diff_mean_yaw/{i}' for i in range(action_dim_yaw)])
        fieldnames.extend([f'diff_logstd_yaw/{i}' for i in range(action_dim_yaw)])
        
        # Hidden state columns (for reference)
        max_hxs_vfvr_dim = 0
        max_hxs_yaw_dim = 0
        for row in log_rows:
            hxs_vfvr, hxs_yaw = extract_rl_hidden_states(row)
            if hxs_vfvr is not None:
                max_hxs_vfvr_dim = max(max_hxs_vfvr_dim, len(hxs_vfvr))
            if hxs_yaw is not None:
                max_hxs_yaw_dim = max(max_hxs_yaw_dim, len(hxs_yaw))
        
        # Note: We'll only log first few hidden states to keep CSV manageable
        # Full hidden states are available in the original log file
        fieldnames.extend([f'hxs_vfvr/{i}' for i in range(min(10, max_hxs_vfvr_dim))])
        fieldnames.extend([f'hxs_yaw/{i}' for i in range(min(10, max_hxs_yaw_dim))])
        
        # Write formatted header
        header_line = ' '.join([format_value(fieldname, 12) for fieldname in fieldnames])
        f.write(header_line + '\n')
        
        # Process each log row
        for row_idx, row in enumerate(log_rows):
            # Extract observations
            obsXY, obsHeading = extract_rl_observations(row)
            if obsXY is None or obsHeading is None:
                print(f"Warning: Row {row_idx} has missing observations, skipping")
                continue
            
            # Extract C++ actions and distributions
            cpp_action_xy, cpp_action_yaw, cpp_mean_vfvr, cpp_logstd_vfvr, cpp_mean_yaw, cpp_logstd_yaw = extract_rl_cpp_actions(row)
            
            # Extract hidden states from log (these are the states BEFORE the inference that produced the logged actions)
            hxs_vfvr_log, hxs_yaw_log = extract_rl_hidden_states(row)
            
            # Set hidden states from log for this inference
            # The hidden states in the log are the states BEFORE the inference
            if hxs_vfvr_log is not None and len(hxs_vfvr_log) > 0:
                hxs_vfvr_tensor = torch.from_numpy(hxs_vfvr_log).unsqueeze(0)
                if hxs_vfvr_tensor.dim() == 1:
                    hxs_vfvr_tensor = hxs_vfvr_tensor.unsqueeze(0)
                policy_vfvr.set_hidden_state(hxs_vfvr_tensor)
            else:
                # If no hidden state in log, reset to zeros (shouldn't happen for RL logs)
                policy_vfvr.reset_hidden_state(batch_size=1, device=torch.device(device))
            
            if hxs_yaw_log is not None and len(hxs_yaw_log) > 0:
                hxs_yaw_tensor = torch.from_numpy(hxs_yaw_log).unsqueeze(0)
                if hxs_yaw_tensor.dim() == 1:
                    hxs_yaw_tensor = hxs_yaw_tensor.unsqueeze(0)
                policy_yaw.set_hidden_state(hxs_yaw_tensor)
            else:
                # If no hidden state in log, reset to zeros (shouldn't happen for RL logs)
                policy_yaw.reset_hidden_state(batch_size=1, device=torch.device(device))
            
            # Get timestamp
            timestamp = row.get('timestamp', str(row_idx))
            
            # Prepare output row
            output_row = {
                'row_index': row_idx,
                'timestamp': timestamp
            }
            
            # Add observations
            for i in range(4):
                output_row[f'obsXY/{i}'] = obsXY[i] if i < len(obsXY) else ''
            for i in range(2):
                output_row[f'obsHeading/{i}'] = obsHeading[i] if i < len(obsHeading) else ''
            
            # Add C++ distributions
            for i in range(action_dim_vfvr):
                output_row[f'cpp_mean_vfvr/{i}'] = cpp_mean_vfvr[i] if cpp_mean_vfvr is not None and i < len(cpp_mean_vfvr) else ''
                output_row[f'cpp_logstd_vfvr/{i}'] = cpp_logstd_vfvr[i] if cpp_logstd_vfvr is not None and i < len(cpp_logstd_vfvr) else ''
            
            for i in range(action_dim_yaw):
                output_row[f'cpp_mean_yaw/{i}'] = cpp_mean_yaw[i] if cpp_mean_yaw is not None and i < len(cpp_mean_yaw) else ''
                output_row[f'cpp_logstd_yaw/{i}'] = cpp_logstd_yaw[i] if cpp_logstd_yaw is not None and i < len(cpp_logstd_yaw) else ''
            
            # Run Python inference for vfvr
            with torch.no_grad():
                obsXY_tensor = torch.from_numpy(obsXY).unsqueeze(0)  # [1, 4]
                action_logits_vfvr, h_next_vfvr = policy_vfvr(obsXY_tensor, normalized=False)
                action_logits_vfvr = action_logits_vfvr.squeeze(0)  # [2*action_dim_vfvr]
                py_mean_vfvr = action_logits_vfvr[:action_dim_vfvr].numpy()
                py_logstd_vfvr = action_logits_vfvr[action_dim_vfvr:].numpy()
            
            # Run Python inference for yaw
            with torch.no_grad():
                obsHeading_tensor = torch.from_numpy(obsHeading).unsqueeze(0)  # [1, 2]
                action_logits_yaw, h_next_yaw = policy_yaw(obsHeading_tensor, normalized=False)
                action_logits_yaw = action_logits_yaw.squeeze(0)  # [2*action_dim_yaw]
                py_mean_yaw = action_logits_yaw[:action_dim_yaw].numpy()
                py_logstd_yaw = action_logits_yaw[action_dim_yaw:].numpy()
            
            # Add Python distributions
            for i in range(action_dim_vfvr):
                output_row[f'py_mean_vfvr/{i}'] = py_mean_vfvr[i]
                output_row[f'py_logstd_vfvr/{i}'] = py_logstd_vfvr[i]
            
            for i in range(action_dim_yaw):
                output_row[f'py_mean_yaw/{i}'] = py_mean_yaw[i]
                output_row[f'py_logstd_yaw/{i}'] = py_logstd_yaw[i]
            
            # Calculate differences
            if cpp_mean_vfvr is not None:
                diff_mean_vfvr = py_mean_vfvr - cpp_mean_vfvr
                for i in range(action_dim_vfvr):
                    output_row[f'diff_mean_vfvr/{i}'] = diff_mean_vfvr[i] if i < len(diff_mean_vfvr) else ''
            else:
                for i in range(action_dim_vfvr):
                    output_row[f'diff_mean_vfvr/{i}'] = ''
            
            if cpp_logstd_vfvr is not None:
                diff_logstd_vfvr = py_logstd_vfvr - cpp_logstd_vfvr
                for i in range(action_dim_vfvr):
                    output_row[f'diff_logstd_vfvr/{i}'] = diff_logstd_vfvr[i] if i < len(diff_logstd_vfvr) else ''
            else:
                for i in range(action_dim_vfvr):
                    output_row[f'diff_logstd_vfvr/{i}'] = ''
            
            if cpp_mean_yaw is not None:
                diff_mean_yaw = py_mean_yaw - cpp_mean_yaw
                for i in range(action_dim_yaw):
                    output_row[f'diff_mean_yaw/{i}'] = diff_mean_yaw[i] if i < len(diff_mean_yaw) else ''
            else:
                for i in range(action_dim_yaw):
                    output_row[f'diff_mean_yaw/{i}'] = ''
            
            if cpp_logstd_yaw is not None:
                diff_logstd_yaw = py_logstd_yaw - cpp_logstd_yaw
            for i in range(action_dim_yaw):
                output_row[f'diff_logstd_yaw/{i}'] = diff_logstd_yaw[i] if i < len(diff_logstd_yaw) else ''
            else:
                for i in range(action_dim_yaw):
                    output_row[f'diff_logstd_yaw/{i}'] = ''
            
            # Add hidden states (first 10 for reference) - these are the states BEFORE inference
            if hxs_vfvr_log is not None:
                for i in range(min(10, len(hxs_vfvr_log))):
                    output_row[f'hxs_vfvr/{i}'] = hxs_vfvr_log[i]
            else:
                for i in range(10):
                    output_row[f'hxs_vfvr/{i}'] = ''
            
            if hxs_yaw_log is not None:
                for i in range(min(10, len(hxs_yaw_log))):
                    output_row[f'hxs_yaw/{i}'] = hxs_yaw_log[i]
            else:
                for i in range(10):
                    output_row[f'hxs_yaw/{i}'] = ''
            
            # Write formatted row
            formatted_row = ' '.join([format_value(output_row.get(fieldname, ''), 12) for fieldname in fieldnames])
            f.write(formatted_row + '\n')
            
            # Progress indicator
            if (row_idx + 1) % 100 == 0:
                print(f"Processed {row_idx + 1}/{len(log_rows)} rows...")
    
    print(f"\nComparison complete! Results written to: {output_file_path}")
    
    # Print summary statistics
    print("\nCalculating summary statistics...")
    with open(output_file_path, 'r') as f:
        # Read header line (first line)
        header_line = f.readline().strip()
        fieldnames = [field.strip() for field in header_line.split()]
        reader = csv.DictReader(f, fieldnames=fieldnames, delimiter=' ', skipinitialspace=True)
        diff_means_vfvr = []
        diff_logstds_vfvr = []
        diff_means_yaw = []
        diff_logstds_yaw = []
        
        for row in reader:
            for i in range(action_dim_vfvr):
                key = f'diff_mean_vfvr/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_means_vfvr.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
                key = f'diff_logstd_vfvr/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_logstds_vfvr.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
            
            for i in range(action_dim_yaw):
                key = f'diff_mean_yaw/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_means_yaw.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
                key = f'diff_logstd_yaw/{i}'
                if key in row and row[key] and row[key].strip():
                    try:
                        diff_logstds_yaw.append(float(row[key]))
                    except (ValueError, TypeError):
                        pass
    
    if diff_means_vfvr:
        print(f"\nVfvr Mean difference statistics:")
        print(f"  Mean: {np.mean(diff_means_vfvr):.6e}")
        print(f"  Std:  {np.std(diff_means_vfvr):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_means_vfvr)):.6e}")
    
    if diff_logstds_vfvr:
        print(f"Vfvr Logstd difference statistics:")
        print(f"  Mean: {np.mean(diff_logstds_vfvr):.6e}")
        print(f"  Std:  {np.std(diff_logstds_vfvr):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_logstds_vfvr)):.6e}")
    
    if diff_means_yaw:
        print(f"\nYaw Mean difference statistics:")
        print(f"  Mean: {np.mean(diff_means_yaw):.6e}")
        print(f"  Std:  {np.std(diff_means_yaw):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_means_yaw)):.6e}")
    
    if diff_logstds_yaw:
        print(f"Yaw Logstd difference statistics:")
        print(f"  Mean: {np.mean(diff_logstds_yaw):.6e}")
        print(f"  Std:  {np.std(diff_logstds_yaw):.6e}")
        print(f"  Max:  {np.max(np.abs(diff_logstds_yaw)):.6e}")
    


def run_inference_test(
    log_file_path: str,
    model_file_path: str,
    output_file_path: str = None,
    nonlinearity: str = 'relu',
    device: str = 'cpu',
    hxs_file_path: str = None,
    hxs_policy_type: str = 'auto'
):
    """Main test function to compare C++ and Python inference.
    
    Args:
        log_file_path: Path to CSV log file from Control.cpp
        model_file_path: Path to PyTorch .pth checkpoint file or JSON file (from pth2json.py)
        output_file_path: Path to output comparison CSV (default: <log_file>_comparison.csv)
        nonlinearity: Activation function ('relu', 'elu', 'tanh')
        device: Device to run inference on ('cpu' or 'cuda')
    """
    policy = load_policy_from_file(model_file_path, nonlinearity, device)
    
    # Determine action dimension from policy
    # The distribution linear outputs [mean, logstd] concatenated, so action_dim = out_features / 2
    action_dim = policy.dist_linear.out_features // 2
    print(f"Action dimension: {action_dim}")
    
    print(f"Reading log file: {log_file_path}")
    log_rows = read_log_file(log_file_path)
    print(f"Found {len(log_rows)} log entries")
    
    if output_file_path is None:
        base_name = os.path.splitext(log_file_path)[0]
        output_file_path = f"{base_name}_comparison.csv"
    
    print(f"Writing comparison results to: {output_file_path}")
    
    # Prepare output CSV
    with open(output_file_path, 'w', newline='') as f:
        # Build header
        fieldnames = ['row_index', 'timestamp']
        
        # Observation columns
        max_obs_dim = 0
        for row in log_rows:
            obs = extract_observations(row)
            if obs is not None:
                max_obs_dim = max(max_obs_dim, len(obs))
        
        for i in range(max_obs_dim):
            fieldnames.append(f'obs/{i}')
        
        # C++ action columns (if available)
        cpp_actions_available = False
        for row in log_rows:
            cpp_actions = extract_cpp_actions(row, action_dim)
            if cpp_actions is not None:
                cpp_actions_available = True
                break
        
        if cpp_actions_available:
            for i in range(action_dim):
                fieldnames.append(f'cpp_action_mean/{i}')
                fieldnames.append(f'cpp_action_logstd/{i}')
        else:
            print("Warning: C++ actions not found in log file. Only Python inference results will be written.")
            print("         To enable C++ action logging, modify Control.cpp to log action_logits.")
        
        # Python action columns
        for i in range(action_dim):
            fieldnames.append(f'py_action_mean/{i}')
            fieldnames.append(f'py_action_logstd/{i}')
        
        # Difference columns (if C++ actions available)
        if cpp_actions_available:
            for i in range(action_dim):
                fieldnames.append(f'diff_mean/{i}')
                fieldnames.append(f'diff_logstd/{i}')
        
        # Write formatted header
        header_line = ' '.join([format_value(fieldname, 12) for fieldname in fieldnames])
        f.write(header_line + '\n')
        
        # Initialize hidden state (from file if provided, otherwise zeros)
        if hxs_file_path:
            hxs_vfvr, hxs_yaw = load_hxs_from_file(hxs_file_path, hxs_policy_type)
            # Use the first non-None hxs found, or default to zeros
            hxs_to_use = hxs_vfvr if hxs_vfvr is not None else hxs_yaw
            if hxs_to_use is not None:
                # Ensure hxs is the right shape: [batch, hidden_size] or [1, batch, hidden_size]
                if hxs_to_use.dim() == 1:
                    hxs_to_use = hxs_to_use.unsqueeze(0)  # [hidden_size] -> [1, hidden_size]
                elif hxs_to_use.dim() == 2 and hxs_to_use.shape[0] != 1:
                    hxs_to_use = hxs_to_use.unsqueeze(0)  # [batch, hidden_size] -> [1, batch, hidden_size]
                
                policy.set_hidden_state(hxs_to_use)
                print(f"Loaded initial hidden state from: {hxs_file_path}")
                if hxs_vfvr is not None:
                    print(f"  Using hxs_vfvr (size: {hxs_vfvr.shape})")
                if hxs_yaw is not None:
                    print(f"  Using hxs_yaw (size: {hxs_yaw.shape})")
            else:
                print(f"Warning: No valid hxs data found in {hxs_file_path}, using zeros")
                policy.reset_hidden_state(batch_size=1, device=torch.device(device))
        else:
            policy.reset_hidden_state(batch_size=1, device=torch.device(device))
        
        # Process each log row
        for row_idx, row in enumerate(log_rows):
            # Extract observation
            obs = extract_observations(row)
            if obs is None:
                print(f"Warning: Row {row_idx} has no observations, skipping")
                continue
            
            # Get timestamp if available
            timestamp = row.get('timestamp', str(row_idx))
            
            # Prepare output row
            output_row = {
                'row_index': row_idx,
                'timestamp': timestamp
            }
            
            # Add observations
            for i in range(max_obs_dim):
                if i < len(obs):
                    output_row[f'obs/{i}'] = obs[i]
                else:
                    output_row[f'obs/{i}'] = ''
            
            # Extract C++ actions if available
            cpp_actions = extract_cpp_actions(row, action_dim)
            if cpp_actions_available:
                if cpp_actions is not None:
                    cpp_mean, cpp_logstd = cpp_actions
                    for i in range(action_dim):
                        output_row[f'cpp_action_mean/{i}'] = cpp_mean[i]
                        output_row[f'cpp_action_logstd/{i}'] = cpp_logstd[i]
                else:
                    # Missing C++ actions for this row
                    for i in range(action_dim):
                        output_row[f'cpp_action_mean/{i}'] = ''
                        output_row[f'cpp_action_logstd/{i}'] = ''
            
            # Run Python inference
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)  # [1, obs_dim]
                action_logits, h_next = policy(obs_tensor, normalized=False)
                
                # action_logits is [1, 2*action_dim] containing [mean, logstd] concatenated
                action_logits = action_logits.squeeze(0)  # [2*action_dim]
                py_mean = action_logits[:action_dim].numpy()
                py_logstd = action_logits[action_dim:].numpy()
            
            # Add Python actions
            for i in range(action_dim):
                output_row[f'py_action_mean/{i}'] = py_mean[i]
                output_row[f'py_action_logstd/{i}'] = py_logstd[i]
            
            # Calculate differences if C++ actions available
            if cpp_actions_available and cpp_actions is not None:
                cpp_mean, cpp_logstd = cpp_actions
                diff_mean = py_mean - cpp_mean
                diff_logstd = py_logstd - cpp_logstd
                
                for i in range(action_dim):
                    output_row[f'diff_mean/{i}'] = diff_mean[i]
                    output_row[f'diff_logstd/{i}'] = diff_logstd[i]
            
            # Write formatted row
            formatted_row = ' '.join([format_value(output_row.get(fieldname, ''), 12) for fieldname in fieldnames])
            f.write(formatted_row + '\n')
            
            # Progress indicator
            if (row_idx + 1) % 100 == 0:
                print(f"Processed {row_idx + 1}/{len(log_rows)} rows...")
    
    print(f"\nComparison complete! Results written to: {output_file_path}")
    
    # Print summary statistics if C++ actions were available
    if cpp_actions_available:
        print("\nCalculating summary statistics...")
        # Re-read the output file to calculate stats
        with open(output_file_path, 'r') as f:
            # Read header line (first line)
            header_line = f.readline().strip()
            fieldnames = [field.strip() for field in header_line.split()]
            reader = csv.DictReader(f, fieldnames=fieldnames, delimiter=' ', skipinitialspace=True)
            diff_means = []
            diff_logstds = []
            for row in reader:
                for i in range(action_dim):
                    mean_key = f'diff_mean/{i}'
                    logstd_key = f'diff_logstd/{i}'
                    if mean_key in row and row[mean_key]:
                        diff_means.append(float(row[mean_key]))
                    if logstd_key in row and row[logstd_key]:
                        diff_logstds.append(float(row[logstd_key]))
        
        if diff_means:
            print(f"Mean difference statistics:")
            print(f"  Mean: {np.mean(diff_means):.6e}")
            print(f"  Std:  {np.std(diff_means):.6e}")
            print(f"  Max:  {np.max(np.abs(diff_means)):.6e}")
            print(f"Logstd difference statistics:")
            print(f"  Mean: {np.mean(diff_logstds):.6e}")
            print(f"  Std:  {np.std(diff_logstds):.6e}")
            print(f"  Max:  {np.max(np.abs(diff_logstds)):.6e}")


def main():
    parser = argparse.ArgumentParser(
        description="Compare C++ and Python inference results from log files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python test_inference.py --log=log_file_rl_vfvr.csv
  python test_inference.py --log=log_file_rl_yaw.csv
  python test_inference.py --log=log_file.csv --output=comparison.csv
        """
    )
    parser.add_argument('--log', type=str, required=True,
                       help='Path to CSV log file from Control.cpp')
    parser.add_argument('--output', type=str, default=None, 
                       help='Output CSV file path (default: <log_file>_comparison.csv)')
    parser.add_argument('--device', type=str, default='cpu', choices=['cpu', 'cuda'],
                       help='Device to run inference on (default: cpu)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.log):
        print(f"Error: Log file not found: {args.log}")
        sys.exit(1)
    
    # Check if it's an RL log file
    is_rl_log = is_rl_log_file(args.log)
    
    if is_rl_log:
        # RL log file - use simplified single-policy processing
        run_inference_test_rl_single(
            args.log,
            args.output,
            args.device
        )
    else:
        # Regular log file - still requires manual policy file specification for backward compatibility
        print("Error: Non-RL log file detected.")
        print("For RL log files, policy file is auto-detected from metadata.")
        print("For non-RL log files, please use the old version of this script or update the log format.")
        sys.exit(1)


if __name__ == '__main__':
    main()
