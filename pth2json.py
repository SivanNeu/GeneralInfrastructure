#!/usr/bin/env python3
"""
Convert PyTorch .pth files to JSON format for C++ consumption.
Stores tensor data along with shape and dtype information.
"""

import torch
import json
import sys
import os
import argparse
from pathlib import Path


def tensor_to_dict(tensor):
    """
    Convert a PyTorch tensor to a dictionary with data, shape, and dtype.
    This format is easier to parse from C++.
    """
    # Ensure tensor is on CPU and convert to list
    if tensor.device.type != 'cpu':
        tensor = tensor.cpu()
    
    return {
        'data': tensor.tolist(),
        'shape': list(tensor.shape),
        'dtype': str(tensor.dtype),
        'numel': int(tensor.numel())  # Total number of elements
    }


def make_json_serializable(obj):
    """
    Recursively convert an object to JSON-serializable format.
    Handles tensors, numpy arrays, and other non-serializable types.
    """
    if isinstance(obj, torch.Tensor):
        return tensor_to_dict(obj)
    elif hasattr(obj, 'item'):  # Handle numpy scalars and similar
        return obj.item()
    elif isinstance(obj, (dict,)):
        return {k: make_json_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [make_json_serializable(item) for item in obj]
    elif isinstance(obj, (int, float, str, bool, type(None))):
        return obj
    else:
        # For other types, try to convert to string representation
        return str(obj)


def pth_to_json(pth_path, json_path=None):
    """
    Convert a PyTorch .pth file to JSON format.
    
    Args:
        pth_path: Path to the .pth file
        json_path: Optional output path. If None, uses pth_path with .json extension
    """
    # Validate input file
    if not os.path.exists(pth_path):
        raise FileNotFoundError(f"PyTorch file not found: {pth_path}")
    
    # Determine output path
    if json_path is None:
        json_path = str(Path(pth_path).with_suffix('.json'))
    
    print(f"Loading PyTorch model from: {pth_path}")
    
    # Load state dict
    try:
        checkpoint = torch.load(pth_path, map_location='cpu')
    except Exception as e:
        raise RuntimeError(f"Failed to load PyTorch file: {e}")
    
    # Handle checkpoints that are wrapped in "model" key (common in Sample Factory)
    if isinstance(checkpoint, dict) and 'model' in checkpoint:
        state_dict = checkpoint['model']
    else:
        state_dict = checkpoint
    
    # Convert tensors to dictionaries with metadata
    weights_dict = {}
    total_params = 0
    
    for key, value in state_dict.items():
        if isinstance(value, torch.Tensor):
            weights_dict[key] = tensor_to_dict(value)
            total_params += value.numel()
        else:
            # Handle non-tensor values (e.g., scalars, strings, nested structures)
            try:
                # Try to make it JSON serializable
                serialized_value = make_json_serializable(value)
                weights_dict[key] = {
                    'data': serialized_value,
                    'type': type(value).__name__
                }
                print(f"Warning: Non-tensor value found for key '{key}': {type(value)}")
            except Exception as e:
                # If serialization fails, store as string representation
                weights_dict[key] = {
                    'data': str(value),
                    'type': type(value).__name__,
                    'error': f"Could not serialize: {e}"
                }
                print(f"Warning: Could not serialize value for key '{key}': {e}")
    
    # Create output dictionary with metadata
    # Ensure all values are JSON serializable
    output = {
        'metadata': {
            'source_file': str(pth_path),
            'num_layers': int(len(weights_dict)),
            'total_parameters': int(total_params)
        },
        'weights': make_json_serializable(weights_dict)
    }
    
    # Save to JSON
    print(f"Saving JSON to: {json_path}")
    try:
        with open(json_path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"Successfully converted {len(weights_dict)} tensors ({total_params:,} parameters)")
    except Exception as e:
        raise RuntimeError(f"Failed to write JSON file: {e}")
    
    return json_path


if __name__ == '__main__':
    # Default path for debugging
    DEFAULT_PTH = '/home/valentin/RL/src/train_dir/rlcat2_yawrate/checkpoint_p0/best_000008610_8816640_reward_4791.792.pth'
    
    parser = argparse.ArgumentParser(
        description='Convert PyTorch .pth files to JSON format for C++ consumption',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Example:\n  python pth2json.py model.pth\n  python pth2json.py model.pth output.json\n  python pth2json.py  # Uses default path for debugging'
    )
    parser.add_argument(
        'pth_path',
        type=str,
        nargs='?',
        default=DEFAULT_PTH,
        help=f'Path to the input .pth file (default: {DEFAULT_PTH})'
    )
    parser.add_argument(
        'json_path',
        type=str,
        nargs='?',
        default=None,
        help='Path to the output .json file (optional, defaults to input filename with .json extension)'
    )
    
    args = parser.parse_args()
    
    try:
        pth_to_json(args.pth_path, args.json_path)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)