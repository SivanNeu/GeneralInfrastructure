"""
Self-contained RL Policy implementation for inference testing.
Based on rl_policyClean.py but simplified for testing purposes.
"""
import sys
import os
import json
import base64

# Fix for Raspberry Pi Zero: disable SVE detection
os.environ['CPUINFO_DISABLE_SVE'] = '1'
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning, message='.*cpuinfo.*')
warnings.filterwarnings('ignore', message='.*prctl.*')

import torch
import torch.nn as nn
import numpy as np

EPS = 1e-5  # Match Sample Factory's _NORM_EPS


class RLPolicyClean(nn.Module):
    """Minimal, inference-only policy that mirrors Sample Factory forward pass.

    Components:
      - Input normalization using checkpoint running mean/var
      - Encoder MLP (Linear/ReLU × 3) for vector obs
      - GRU core
      - Linear distribution head producing [mean, logstd]

    Forward returns (action_logits, new_hxs) where action_logits = [mean, logstd].
    """

    def __init__(self):
        super().__init__()
        self.obs_mean: torch.Tensor | None = None
        self.obs_var: torch.Tensor | None = None
        self.encoder: nn.Module | None = None
        self.core: nn.GRU | None = None
        self.decoder: nn.Module | None = None
        self.dist_linear: nn.Linear | None = None
        self.hxs: torch.Tensor | None = None  # shape [num_layers, batch, hidden]

    def forward(self, obs: torch.Tensor, *, normalized: bool = False):
        """Forward pass through the policy network.
        
        Args:
            obs: Observation tensor [batch, features] or [features]
            normalized: If True, skip normalization (obs already normalized)
            
        Returns:
            action_logits: [batch, 2*action_dim] containing [mean, logstd] concatenated
            h_next: New hidden state [batch, hidden_size]
        """
        obs = torch.tensor(obs, dtype=torch.float32)
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)  # [features] -> [1, features]
        
        # Normalize observation
        if not normalized:
            x = (obs - self.obs_mean) / torch.sqrt(self.obs_var + EPS)
            x = torch.clamp(x, -5.0, 5.0)
        else:
            x = obs

        # Encoder forward (standard PyTorch Sequential)
        x = self.encoder(x)  # [batch, encoder_out]

        # GRU forward (standard PyTorch GRU)
        # GRU expects [seq_len, batch, features]; hxs is [num_layers, batch, hidden]
        x = x.unsqueeze(0)  # [batch, features] -> [1, batch, features]
        if self.hxs is None or self.hxs.shape[1] != x.shape[1]:
            self.hxs = torch.zeros(1, x.shape[1], self.core.hidden_size, device=x.device)
        
        # Standard PyTorch GRU forward
        x, h_next = self.core(x, self.hxs)
        x = x.squeeze(0)  # [1, batch, hidden_size] -> [batch, hidden_size]
        self.hxs = h_next

        # Decoder forward (standard PyTorch module)
        if self.decoder is not None:
            x = self.decoder(x)

        # Distribution linear forward (standard PyTorch Linear)
        params = self.dist_linear(x)  # [batch, 2*action_dim]
        
        # Split into mean and logstd, then concatenate (standard PyTorch operations)
        mean, logstd = params.chunk(2, dim=-1)
        action_logits = torch.cat([mean, logstd], dim=-1)
        
        # Return h_next in [batch, hidden_size] format for consistency with C++
        h_next = h_next.squeeze(0)  # [1, batch, hidden_size] -> [batch, hidden_size]
        return action_logits, h_next

    def _init_modules(self, obs_mean: torch.Tensor, obs_var: torch.Tensor,
                       encoder: nn.Module, core: nn.GRU, dist_linear: nn.Linear,
                       decoder: nn.Module | None = None):
        self.obs_mean = nn.Parameter(obs_mean.float(), requires_grad=False)
        self.obs_var = nn.Parameter(obs_var.float(), requires_grad=False)
        self.encoder = encoder
        self.core = core
        self.decoder = decoder
        self.dist_linear = dist_linear
        self.hxs = None

    def reset_hidden_state(self, batch_size: int = 1, device: torch.device | None = None) -> None:
        """Reset internal RNN state to zeros."""
        if device is None:
            device = next(self.parameters()).device
        self.hxs = torch.zeros(1, batch_size, self.core.hidden_size, device=device)

    def set_hidden_state(self, hxs: torch.Tensor) -> None:
        """Set internal RNN state.
        
        Args:
            hxs: Hidden state tensor of shape [batch, hidden_size] or [1, batch, hidden_size]
        """
        if hxs.dim() == 2:
            self.hxs = hxs.unsqueeze(0).clone()
        else:
            self.hxs = hxs.clone()

    @staticmethod
    def _activation(nonlinearity: str) -> nn.Module:
        if nonlinearity == 'relu':
            return nn.ReLU(inplace=False)
        if nonlinearity == 'elu':
            return nn.ELU(inplace=False)
        if nonlinearity == 'tanh':
            return nn.Tanh()
        raise RuntimeError(f"Unsupported nonlinearity: {nonlinearity}")

    @staticmethod
    def _find_mlp_linear_indices(ckpt: dict) -> list[int]:
        """Find ordered Linear layer indices inside encoder.encoders.obs.mlp_head.*"""
        prefix = 'encoder.encoders.obs.mlp_head.'
        linear_indices: set[int] = set()
        for k in ckpt.keys():
            if not k.startswith(prefix):
                continue
            if k.endswith('.weight'):
                try:
                    idx = int(k[len(prefix):].split('.')[0])
                except Exception:
                    continue
                linear_indices.add(idx)
        return sorted(linear_indices)

    @classmethod
    def _build_encoder_from_ckpt(cls, ckpt: dict, *, nonlinearity: str, jit: bool) -> nn.Module:
        act = cls._activation(nonlinearity)
        indices = cls._find_mlp_linear_indices(ckpt)
        if len(indices) == 0:
            return nn.Identity()

        layers: list[nn.Module] = []
        for i, idx in enumerate(indices):
            w_key = f'encoder.encoders.obs.mlp_head.{idx}.weight'
            b_key = f'encoder.encoders.obs.mlp_head.{idx}.bias'
            weight = ckpt[w_key]
            bias = ckpt[b_key]
            in_f, out_f = weight.shape[1], weight.shape[0]
            lin = nn.Linear(in_f, out_f)
            lin.weight.data.copy_(weight)
            lin.bias.data.copy_(bias)
            layers.append(lin)
            layers.append(act)

        enc = nn.Sequential(*layers)
        if jit:
            enc = torch.jit.script(enc)
        return enc

    @classmethod
    def load_from_checkpoint(
        cls,
        path: str,
        device: str = 'cpu',
        *,
        nonlinearity: str = 'relu',
        jit_encoder: bool = False,
    ) -> 'RLPolicyClean':
        """Create a policy instance from a Sample Factory checkpoint (.pth).

        Args:
            path: checkpoint path (best_*.pth or checkpoint_*.pth)
            device: 'cpu' or 'cuda'
            nonlinearity: 'relu' | 'elu' | 'tanh' — must match training config
            jit_encoder: if True, torch.jit.script the encoder MLP
        """
        ckpt = torch.load(path, map_location=device, weights_only=False)["model"]

        # Observation normalizer parameters
        mean_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_mean'
        var_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_var'
        if mean_key in ckpt and var_key in ckpt:
            obs_mean = ckpt[mean_key].detach()
            obs_var = ckpt[var_key].detach()
        else:
            # Fallback: infer input size from the first linear layer
            indices = [0]
            try:
                indices = RLPolicyClean._find_mlp_linear_indices(ckpt)
            except Exception:
                pass
            if len(indices) > 0:
                first_w = ckpt[f'encoder.encoders.obs.mlp_head.{indices[0]}.weight']
                in_size = first_w.shape[1]
            else:
                in_size = 1
            obs_mean = torch.zeros(in_size)
            obs_var = torch.ones(in_size)

        # Build encoder
        encoder = cls._build_encoder_from_ckpt(ckpt, nonlinearity=nonlinearity, jit=jit_encoder)

        # Determine GRU input size
        gru_input_size = 512
        lin_indices = cls._find_mlp_linear_indices(ckpt)
        if len(lin_indices) > 0:
            last_idx = lin_indices[-1]
            last_w = ckpt[f'encoder.encoders.obs.mlp_head.{last_idx}.weight']
            gru_input_size = last_w.shape[0]

        # GRU core
        rnn_size = ckpt['core.core.weight_hh_l0'].shape[1]
        core = nn.GRU(input_size=gru_input_size, hidden_size=rnn_size, batch_first=False)
        core.weight_ih_l0.data.copy_(ckpt['core.core.weight_ih_l0'])
        core.weight_hh_l0.data.copy_(ckpt['core.core.weight_hh_l0'])
        core.bias_ih_l0.data.copy_(ckpt['core.core.bias_ih_l0'])
        core.bias_hh_l0.data.copy_(ckpt['core.core.bias_hh_l0'])

        # Identity decoder
        class _Identity(nn.Module):
            def forward(self, x):
                return x

        decoder = _Identity()

        # Distribution head: [mean, logstd]
        dist_in = ckpt['action_parameterization.distribution_linear.weight'].shape[1]
        num_of_actions = ckpt['action_parameterization.distribution_linear.weight'].shape[0]
        dist_linear = nn.Linear(dist_in, num_of_actions)
        dist_linear.weight.data.copy_(ckpt['action_parameterization.distribution_linear.weight'])
        dist_linear.bias.data.copy_(ckpt['action_parameterization.distribution_linear.bias'])

        policy = cls().to(device)
        policy._init_modules(obs_mean, obs_var, encoder, core, dist_linear, decoder)
        return policy

    @staticmethod
    def _decode_base64_tensor(tensor_dict: dict) -> torch.Tensor:
        """Decode tensor data from JSON format (supports both base64 and ASCII array formats).
        
        Args:
            tensor_dict: Dictionary with 'data' (base64 string or list of numbers), 'shape', 'dtype', 'encoding'
            
        Returns:
            PyTorch tensor
        """
        encoding = tensor_dict.get('encoding', None)
        data = tensor_dict['data']
        
        # Get dtype and determine numpy dtype
        dtype_str = tensor_dict.get('dtype', 'float32')
        if 'float64' in dtype_str or 'double' in dtype_str:
            np_dtype = np.float64
        elif 'float32' in dtype_str or 'float' in dtype_str:
            np_dtype = np.float32
        elif 'float16' in dtype_str or 'half' in dtype_str:
            np_dtype = np.float16
        else:
            np_dtype = np.float32  # default
        
        # Get shape
        shape = tuple(tensor_dict['shape'])
        
        # Handle base64 encoding
        if encoding == 'base64':
            # Decode base64 string
            decoded_bytes = base64.b64decode(data)
            # Convert bytes to numpy array
            num_elements = int(np.prod(shape))
            element_size = np.dtype(np_dtype).itemsize
            
            if len(decoded_bytes) != num_elements * element_size:
                raise ValueError(f"Size mismatch: expected {num_elements * element_size} bytes, got {len(decoded_bytes)}")
            
            np_array = np.frombuffer(decoded_bytes, dtype=np_dtype).reshape(shape)
        else:
            # Handle ASCII/array format - data is a list of numbers
            if isinstance(data, list):
                np_array = np.array(data, dtype=np_dtype)
            elif isinstance(data, str):
                # Try to parse as JSON array string
                import json
                data_list = json.loads(data)
                np_array = np.array(data_list, dtype=np_dtype)
            else:
                raise ValueError(f"Unsupported data format. Expected list or base64 string, got {type(data)}")
            
            # Reshape to match expected shape
            if np_array.shape != shape:
                np_array = np_array.reshape(shape)
        
        # Convert to PyTorch tensor
        return torch.from_numpy(np_array.copy())

    @classmethod
    def load_from_json(
        cls,
        json_path: str,
        device: str = 'cpu',
        *,
        nonlinearity: str = 'relu',
    ) -> 'RLPolicyClean':
        """Create a policy instance from a JSON file (created by pth2json.py).

        Args:
            json_path: Path to JSON file
            device: 'cpu' or 'cuda'
            nonlinearity: 'relu' | 'elu' | 'tanh' — must match training config
        """
        with open(json_path, 'r') as f:
            json_data = json.load(f)
        
        weights = json_data.get('weights', {})
        
        # Observation normalizer parameters
        mean_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_mean'
        var_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_var'
        
        if mean_key in weights and var_key in weights:
            obs_mean = cls._decode_base64_tensor(weights[mean_key])
            obs_var = cls._decode_base64_tensor(weights[var_key])
        else:
            # Fallback: infer input size from the first linear layer
            indices = cls._find_mlp_linear_indices_from_json(weights)
            if len(indices) > 0:
                first_w_key = f'encoder.encoders.obs.mlp_head.{indices[0]}.weight'
                first_w = cls._decode_base64_tensor(weights[first_w_key])
                in_size = first_w.shape[1]
            else:
                in_size = 1
            obs_mean = torch.zeros(in_size)
            obs_var = torch.ones(in_size)

        # Build encoder
        encoder = cls._build_encoder_from_json(weights, nonlinearity=nonlinearity)

        # Determine GRU input size
        gru_input_size = 512
        lin_indices = cls._find_mlp_linear_indices_from_json(weights)
        if len(lin_indices) > 0:
            last_idx = lin_indices[-1]
            last_w_key = f'encoder.encoders.obs.mlp_head.{last_idx}.weight'
            last_w = cls._decode_base64_tensor(weights[last_w_key])
            gru_input_size = last_w.shape[0]

        # GRU core
        rnn_size = cls._decode_base64_tensor(weights['core.core.weight_hh_l0']).shape[1]
        core = nn.GRU(input_size=gru_input_size, hidden_size=rnn_size, batch_first=False)
        core.weight_ih_l0.data.copy_(cls._decode_base64_tensor(weights['core.core.weight_ih_l0']))
        core.weight_hh_l0.data.copy_(cls._decode_base64_tensor(weights['core.core.weight_hh_l0']))
        core.bias_ih_l0.data.copy_(cls._decode_base64_tensor(weights['core.core.bias_ih_l0']))
        core.bias_hh_l0.data.copy_(cls._decode_base64_tensor(weights['core.core.bias_hh_l0']))

        # Identity decoder
        class _Identity(nn.Module):
            def forward(self, x):
                return x

        decoder = _Identity()

        # Distribution head: [mean, logstd]
        dist_w = cls._decode_base64_tensor(weights['action_parameterization.distribution_linear.weight'])
        dist_b = cls._decode_base64_tensor(weights['action_parameterization.distribution_linear.bias'])
        dist_in = dist_w.shape[1]
        num_of_actions = dist_w.shape[0]
        dist_linear = nn.Linear(dist_in, num_of_actions)
        dist_linear.weight.data.copy_(dist_w)
        dist_linear.bias.data.copy_(dist_b)

        policy = cls().to(device)
        policy._init_modules(obs_mean, obs_var, encoder, core, dist_linear, decoder)
        return policy

    @staticmethod
    def _find_mlp_linear_indices_from_json(weights: dict) -> list[int]:
        """Find ordered Linear layer indices from JSON weights dict."""
        prefix = 'encoder.encoders.obs.mlp_head.'
        linear_indices: set[int] = set()
        for k in weights.keys():
            if not k.startswith(prefix):
                continue
            if k.endswith('.weight'):
                try:
                    idx = int(k[len(prefix):].split('.')[0])
                except Exception:
                    continue
                linear_indices.add(idx)
        return sorted(linear_indices)

    @classmethod
    def _build_encoder_from_json(cls, weights: dict, *, nonlinearity: str) -> nn.Module:
        """Build encoder from JSON weights dict."""
        act = cls._activation(nonlinearity)
        indices = cls._find_mlp_linear_indices_from_json(weights)
        if len(indices) == 0:
            return nn.Identity()

        layers: list[nn.Module] = []
        for i, idx in enumerate(indices):
            w_key = f'encoder.encoders.obs.mlp_head.{idx}.weight'
            b_key = f'encoder.encoders.obs.mlp_head.{idx}.bias'
            weight = cls._decode_base64_tensor(weights[w_key])
            bias = cls._decode_base64_tensor(weights[b_key])
            in_f, out_f = weight.shape[1], weight.shape[0]
            lin = nn.Linear(in_f, out_f)
            lin.weight.data.copy_(weight)
            lin.bias.data.copy_(bias)
            layers.append(lin)
            layers.append(act)

        return nn.Sequential(*layers)
