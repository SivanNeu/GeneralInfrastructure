import sys
# Fix for Raspberry Pi Zero: disable SVE detection to avoid prctl(PR_SVE_GET_VL) error
import os
# Force disable SVE detection before any imports
os.environ['CPUINFO_DISABLE_SVE'] = '1'
# Suppress cpuinfo errors by redirecting stderr during torch import
from copy import deepcopy
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning, message='.*cpuinfo.*')
warnings.filterwarnings('ignore', message='.*prctl.*')

# Redirect stderr at file descriptor level to suppress cpuinfo error messages
# This catches errors from C extensions that write directly to fd 2
_saved_stderr_fd = None
_devnull_fd = None
_original_stderr = None
_original_stderr_fd = None
try:
    _original_stderr_fd = sys.stderr.fileno()
    # Save a copy of the original stderr fd
    _saved_stderr_fd = os.dup(_original_stderr_fd)
    # Open /dev/null
    _devnull_fd = os.open(os.devnull, os.O_WRONLY)
    # Redirect stderr to /dev/null
    os.dup2(_devnull_fd, _original_stderr_fd)
except (AttributeError, OSError):
    # Fallback to Python-level redirection if fd manipulation fails
    _original_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
finally:
    # Restore stderr
    try:
        if _saved_stderr_fd is not None and _original_stderr_fd is not None:
            # Restore original stderr
            os.dup2(_saved_stderr_fd, _original_stderr_fd)
            os.close(_saved_stderr_fd)
            if _devnull_fd is not None:
                os.close(_devnull_fd)
        elif _original_stderr is not None:
            sys.stderr.close()
            sys.stderr = _original_stderr
    except Exception:
        pass  # Ignore errors during cleanup

# Ensure we can import Sample Factory utilities if needed
sys.path.insert(0, 'sample-factory')


EPS = 1e-5  # Match Sample Factory's _NORM_EPS from running_mean_std.py


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
        self.decoder: nn.Module | None = None  # kept for parity; identity by default
        self.dist_linear: nn.Linear | None = None
        # Internal recurrent state (not exposed to user)
        self.hxs: torch.Tensor | None = None  # shape [num_layers, batch, hidden]

    def forward(self, obs: torch.Tensor, *, normalized: bool = False):
        # Normalize exactly like SF when inputs are raw
        obs = torch.tensor(obs, dtype=torch.float32)
        # Ensure observation has batch dimension [batch, features]
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)  # [features] -> [1, features]
        if not normalized:
            x = (obs - self.obs_mean) / torch.sqrt(self.obs_var + EPS)
            x = torch.clamp(x, -5.0, 5.0)
        else:
            x = obs

        # Encoder MLP expects [batch, features]
        x = self.encoder(x)  # [batch, encoder_out]

        # GRU expects [seq_len, batch, features]; hxs is [num_layers, batch, hidden]
        x = x.unsqueeze(0)  # [batch, features] -> [1, batch, features]
        if self.hxs is None or self.hxs.shape[1] != x.shape[1]:
            self.hxs = torch.zeros(1, x.shape[1], self.core.hidden_size, device=x.device)
        
        # Create deepcopies for both calculations to ensure fair comparison
        x_copy = deepcopy(x)
        hxs_copy = deepcopy(self.hxs)
        
        # Verify inputs are the same
        if not torch.allclose(x, x_copy, atol=1e-8):
            print("WARNING: x and x_copy differ!")
        if not torch.allclose(self.hxs, hxs_copy, atol=1e-8):
            print("WARNING: hxs and hxs_copy differ!")
        
        # Debug: Print input values
        print(f"DEBUG: Input x shape: {x.shape}, first 5 values: {x[0, 0, :5]}")
        print(f"DEBUG: Hidden state hxs shape: {self.hxs.shape}, first 5 values: {self.hxs[0, 0, :5]}")
        
        # PyTorch GRU call (implicit calculation)
        # Note: nn.GRU expects [seq_len, batch, input_size] and returns [seq_len, batch, hidden_size]
        x2, h_next2 = self.core(x_copy, hxs_copy)
        
        # Squeeze PyTorch output for comparison: [seq_len, batch, hidden_size] -> [batch, hidden_size]
        x2_squeezed = x2.squeeze(0)  # [1, batch, hidden_size] -> [batch, hidden_size]
        
        # Try using GRUCell directly to see if it matches
        # This will help us understand what PyTorch is actually doing
        try:
            gru_cell = nn.GRUCell(input_size=self.core.input_size, hidden_size=self.core.hidden_size)
            gru_cell.weight_ih.data = self.core.weight_ih_l0.clone()
            gru_cell.weight_hh.data = self.core.weight_hh_l0.clone()
            gru_cell.bias_ih.data = self.core.bias_ih_l0.clone()
            gru_cell.bias_hh.data = self.core.bias_hh_l0.clone()
            
            # GRUCell expects [batch, input_size] and [batch, hidden_size]
            x_cell_input = x_copy[0].clone()  # [batch, input_size]
            h_cell_input = hxs_copy[0].clone()  # [batch, hidden_size]
            
            # Verify inputs match
            print(f"  GRUCell input check:")
            print(f"  x_cell_input[0,:5]={x_cell_input[0,:5]}")
            print(f"  h_cell_input[0,:5]={h_cell_input[0,:5]}")
            
            h_cell_output = gru_cell(x_cell_input, h_cell_input)
            
            # Compare GRUCell with nn.GRU
            diff_cell = torch.abs(h_cell_output - x2_squeezed)
            print(f"  GRUCell vs nn.GRU max diff: {torch.max(diff_cell).item():.2e}")
            if torch.max(diff_cell) > 1e-5:
                print(f"  WARNING: GRUCell and nn.GRU differ!")
                print(f"  GRUCell[0,0]={h_cell_output[0,0]:.4f}, nn.GRU[0,0]={x2_squeezed[0,0]:.4f}")
            else:
                print(f"  ✓ GRUCell matches nn.GRU, using GRUCell output as reference")
                # Use GRUCell output as the "correct" reference
                x2_squeezed = h_cell_output
                print(f"  GRUCell output[0,0]={h_cell_output[0,0]:.4f} (this is what PyTorch gets)")
        except Exception as e:
            print(f"  Could not test GRUCell: {e}")
        
        # Debug: Check what PyTorch GRU actually used/received
        print(f"DEBUG: PyTorch GRU output x2 shape: {x2.shape}, first 5 values: {x2[0, 0, :5]}")
        print(f"DEBUG: PyTorch GRU output h_next2 shape: {h_next2.shape}, first 5 values: {h_next2[0, 0, :5]}")
        
        # Also try using functional GRU for comparison
        # This uses the same weights but might help debug
        try:
            # Extract weights in the format F.gru expects
            weight_ih_flat = self.core.weight_ih_l0  # [3*hidden, input]
            weight_hh_flat = self.core.weight_hh_l0  # [3*hidden, hidden]
            bias_ih_flat = self.core.bias_ih_l0 if self.core.bias else None
            bias_hh_flat = self.core.bias_hh_l0 if self.core.bias else None
            
            # F.gru expects weights as a tuple of (weight_ih, weight_hh)
            # and biases as a tuple of (bias_ih, bias_hh) or None
            weights = (weight_ih_flat, weight_hh_flat)
            biases = (bias_ih_flat, bias_hh_flat) if bias_ih_flat is not None else None
            
            x3, h_next3 = F.gru(x_copy, hxs_copy, weights, biases, 
                               num_layers=1, dropout=0.0, training=False, 
                               bidirectional=False, batch_first=False)
            
            # Compare F.gru with nn.GRU
            if not torch.allclose(x2, x3, atol=1e-6):
                print(f"WARNING: F.gru and nn.GRU differ! Max diff: {torch.max(torch.abs(x2 - x3)).item():.2e}")
        except Exception as e:
            print(f"Could not test F.gru: {e}")
        
        # EXPLICIT GRU CALCULATION - matches C++ implementation in RLPolicyJSON.cpp
        # This allows line-by-line comparison with the C++ code
        # GRU computation: h_t = (1 - z_t) * h_{t-1} + z_t * n_t
        # where z_t = sigmoid(W_z @ x + U_z @ h + b_z)  [update gate]
        #       r_t = sigmoid(W_r @ x + U_r @ h + b_r)  [reset gate]
        #       n_t = tanh(W_n @ x + b_n_ih + r_t * (U_n @ h + b_n_hh))  [new gate]
        
        # Use the same inputs as PyTorch GRU for fair comparison
        x_explicit = deepcopy(x_copy)  # Use same input as PyTorch call (x_copy, not x)
        hxs_explicit = deepcopy(hxs_copy)  # Use same hidden state as PyTorch call (hxs_copy, not self.hxs)
        
        # Verify inputs match
        if not torch.allclose(x_explicit, x_copy, atol=1e-8):
            print(f"  WARNING: x_explicit and x_copy differ!")
        if not torch.allclose(hxs_explicit, hxs_copy, atol=1e-8):
            print(f"  WARNING: hxs_explicit and hxs_copy differ!")
        print(f"  ✓ Using same inputs as PyTorch GRU")
        
        # Extract weights and biases from GRU
        # IMPORTANT: Get these BEFORE calling PyTorch GRU, as it might modify them
        weight_ih = self.core.weight_ih_l0.clone()  # [3*hidden_size, input_size]
        weight_hh = self.core.weight_hh_l0.clone()  # [3*hidden_size, hidden_size]
        bias_ih = self.core.bias_ih_l0.clone()  # [3*hidden_size]
        bias_hh = self.core.bias_hh_l0.clone()  # [3*hidden_size]
        
        # Debug: Check if weights match what PyTorch sees
        print(f"  Weight check:")
        print(f"  weight_ih shape: {weight_ih.shape}, first element: {weight_ih[0,0]:.6f}")
        print(f"  weight_hh shape: {weight_hh.shape}, first element: {weight_hh[0,0]:.6f}")
        print(f"  bias_ih shape: {bias_ih.shape}, first element: {bias_ih[0]:.6f}")
        print(f"  bias_hh shape: {bias_hh.shape}, first element: {bias_hh[0]:.6f}")
        
        # Verify these match the GRU's current weights (after PyTorch call)
        weight_ih_after = self.core.weight_ih_l0
        if not torch.allclose(weight_ih, weight_ih_after, atol=1e-6):
            print(f"  WARNING: weight_ih changed after PyTorch GRU call!")
        else:
            print(f"  ✓ Weights unchanged after PyTorch GRU call")
        
        hidden_size = self.core.hidden_size
        input_size = x_explicit.shape[2]  # [1, batch, input_size]
        batch_size = x_explicit.shape[1]
        
        # Current hidden state: [1, batch, hidden_size] -> [batch, hidden_size]
        h_prev = hxs_explicit[0]  # [batch, hidden_size]
        
        # Input: [1, batch, input_size] -> [batch, input_size]
        x_input = x_explicit[0]  # [batch, input_size]
        
        # Debug: Verify what we're using matches what PyTorch sees
        print(f"  Input verification:")
        print(f"  x_input[0,:5]={x_input[0,:5]}")
        print(f"  h_prev[0,:5]={h_prev[0,:5]}")
        print(f"  x_copy[0,0,:5]={x_copy[0,0,:5]}")
        print(f"  hxs_copy[0,0,:5]={hxs_copy[0,0,:5]}")
        if not torch.allclose(x_input, x_copy[0], atol=1e-6):
            print(f"  WARNING: x_input doesn't match x_copy[0]!")
        if not torch.allclose(h_prev, hxs_copy[0], atol=1e-6):
            print(f"  WARNING: h_prev doesn't match hxs_copy[0]!")
        
        # Split weights into r, z, n gates (each is hidden_size rows)
        # IMPORTANT: PyTorch GRU gate order is [reset, update, new], NOT [update, reset, new]!
        # Gate order: [r_gate, z_gate, n_gate]
        W_r = weight_ih[0:hidden_size, :]  # [hidden_size, input_size] - RESET gate
        W_z = weight_ih[hidden_size:2*hidden_size, :]  # [hidden_size, input_size] - UPDATE gate
        W_n = weight_ih[2*hidden_size:3*hidden_size, :]  # [hidden_size, input_size] - NEW gate
        
        U_r = weight_hh[0:hidden_size, :]  # [hidden_size, hidden_size] - RESET gate
        U_z = weight_hh[hidden_size:2*hidden_size, :]  # [hidden_size, hidden_size] - UPDATE gate
        U_n = weight_hh[2*hidden_size:3*hidden_size, :]  # [hidden_size, hidden_size] - NEW gate
        
        # Split biases into r, z, n gates (same order: reset, update, new)
        b_r_ih = bias_ih[0:hidden_size]  # [hidden_size] - RESET gate
        b_z_ih = bias_ih[hidden_size:2*hidden_size]  # [hidden_size] - UPDATE gate
        b_n_ih = bias_ih[2*hidden_size:3*hidden_size]  # [hidden_size] - NEW gate
        
        b_r_hh = bias_hh[0:hidden_size]  # [hidden_size] - RESET gate
        b_z_hh = bias_hh[hidden_size:2*hidden_size]  # [hidden_size] - UPDATE gate
        b_n_hh = bias_hh[2*hidden_size:3*hidden_size]  # [hidden_size] - NEW gate
        
        # Compute gates for each sample in batch
        # z_t = W_z @ x + U_z @ h_prev
        z_t = torch.matmul(x_input, W_z.t())  # [batch, hidden_size]
        z_h = torch.matmul(h_prev, U_z.t())  # [batch, hidden_size]
        
        # r_t = W_r @ x + U_r @ h_prev
        r_t = torch.matmul(x_input, W_r.t())  # [batch, hidden_size]
        r_h = torch.matmul(h_prev, U_r.t())  # [batch, hidden_size]
        
        # n_t = W_n @ x
        n_t = torch.matmul(x_input, W_n.t())  # [batch, hidden_size]
        # n_h = U_n @ h_prev
        n_h = torch.matmul(h_prev, U_n.t())  # [batch, hidden_size]
        
        # Add biases and apply activations
        # For z and r gates: sigmoid(W @ x + U @ h + b_ih + b_hh)
        # Note: PyTorch adds biases element-wise, so we need to broadcast correctly
        z_pre_sigmoid = z_t + z_h + b_z_ih.unsqueeze(0) + b_z_hh.unsqueeze(0)
        z = torch.sigmoid(z_pre_sigmoid)  # [batch, hidden_size]
        
        r_pre_sigmoid = r_t + r_h + b_r_ih.unsqueeze(0) + b_r_hh.unsqueeze(0)
        r = torch.sigmoid(r_pre_sigmoid)  # [batch, hidden_size]
        
        # For n gate: tanh(W_n @ x + b_n_ih + r * (U_n @ h + b_n_hh))
        # This matches the C++ implementation: tanh(n_t[i] + b_n_ih[i] + r[i] * (n_h[i] + b_n_hh[i]))
        # IMPORTANT: The order matters! PyTorch computes: tanh(W_in @ x + b_in + r * (W_hn @ h + b_hn))
        n_pre_tanh = n_t + b_n_ih.unsqueeze(0) + r * (n_h + b_n_hh.unsqueeze(0))
        n = torch.tanh(n_pre_tanh)  # [batch, hidden_size]
        
        # Debug: Check intermediate gate values and verify manually
        print(f"  Gate computation verification:")
        print(f"  z_t[0,0]={z_t[0,0]:.4f}, z_h[0,0]={z_h[0,0]:.4f}")
        print(f"  b_z_ih[0]={b_z_ih[0]:.4f}, b_z_hh[0]={b_z_hh[0]:.4f}")
        z_sum = z_t[0,0] + z_h[0,0] + b_z_ih[0] + b_z_hh[0]
        print(f"  z_sum={z_sum:.4f}, sigmoid(z_sum)={torch.sigmoid(torch.tensor(z_sum)):.4f}, z[0,0]={z[0,0]:.4f}")
        
        print(f"  n_t[0,0]={n_t[0,0]:.4f}, n_h[0,0]={n_h[0,0]:.4f}, r[0,0]={r[0,0]:.4f}")
        print(f"  b_n_ih[0]={b_n_ih[0]:.4f}, b_n_hh[0]={b_n_hh[0]:.4f}")
        n_sum = n_t[0,0] + b_n_ih[0] + r[0,0] * (n_h[0,0] + b_n_hh[0])
        print(f"  n_sum={n_sum:.4f}, tanh(n_sum)={torch.tanh(torch.tensor(n_sum)):.4f}, n[0,0]={n[0,0]:.4f}")
        
        # Update hidden state
        # The standard GRU formula is: h_t = (1 - z_t) * h_{t-1} + z_t * n_t
        # When h_{t-1} = 0, this becomes: h_t = z_t * n_t
        # But wait - let's check PyTorch's actual formula
        # PyTorch GRU uses: h_t = (1 - z_t) * n_t + z_t * h_{t-1}
        # When h_{t-1} = 0: h_t = (1 - z_t) * n_t
        
        # Try both formulas
        h_next_standard = (1.0 - z) * h_prev + z * n  # Standard: (1-z)*h + z*n
        h_next_pytorch_style = (1.0 - z) * n + z * h_prev  # PyTorch style: (1-z)*n + z*h
        
        # When h_prev=0:
        # Standard: h = z*n
        # PyTorch: h = (1-z)*n
        
        # Check which one matches
        diff_standard = torch.abs(h_next_standard - x2_squeezed)
        diff_pytorch = torch.abs(h_next_pytorch_style - x2_squeezed)
        
        print(f"  Formula test:")
        print(f"  Standard formula h[0,0]={h_next_standard[0,0]:.4f}, diff={diff_standard[0,0]:.4f}")
        print(f"  PyTorch style h[0,0]={h_next_pytorch_style[0,0]:.4f}, diff={diff_pytorch[0,0]:.4f}")
        print(f"  Expected PyTorch: {x2_squeezed[0,0]:.4f}")
        
        # Use the one that matches better
        if torch.max(diff_pytorch) < torch.max(diff_standard):
            print(f"  Using PyTorch style formula")
            h_next = h_next_pytorch_style
        else:
            print(f"  Using standard formula (but neither matches well)")
            h_next = h_next_standard
        
        # Now let's manually compute what PyTorch should get
        # Using GRU formula: h = (1-z)*h_prev + z*n
        h_manual = (1.0 - z[0,0]) * h_prev[0,0] + z[0,0] * n[0,0]
        print(f"  Manual h[0,0] = (1-{z[0,0]:.4f})*{h_prev[0,0]:.4f} + {z[0,0]:.4f}*{n[0,0]:.4f} = {h_manual:.4f}")
        print(f"  Our h_next[0,0]={h_next[0,0]:.4f}, PyTorch x2[0,0]={x2_squeezed[0,0]:.4f}")
        
        # Debug: Verify the computation
        # When h_prev = 0, h_next should equal z * n
        h_next_from_z_n = z * n
        if torch.allclose(h_prev, torch.zeros_like(h_prev), atol=1e-6):
            print(f"  Since h_prev=0, h_next should equal z*n")
            print(f"  h_next[0,0]={h_next[0,0]:.4f}, (z*n)[0,0]={h_next_from_z_n[0,0]:.4f}")
            print(f"  Difference: {torch.abs(h_next - h_next_from_z_n)[0,0]:.4f}")
        
        # Compare with PyTorch output
        diff = torch.abs(h_next - x2_squeezed)
        print(f"  Our h_next[0,0]={h_next[0,0]:.4f}, PyTorch x2[0,0]={x2_squeezed[0,0]:.4f}")
        print(f"  Difference: {diff[0,0]:.4f}")
        
        # Check if maybe PyTorch is using a different computation
        # Try: h = n + z * (h_prev - n) which is equivalent to (1-z)*n + z*h_prev when h_prev=0
        h_next_alt = n + z * (h_prev - n)
        diff_alt = torch.abs(h_next_alt - x2_squeezed)
        print(f"  Alternative formula h_next_alt[0,0]={h_next_alt[0,0]:.4f}, diff={diff_alt[0,0]:.4f}")
        
        # Use the one that matches better
        if torch.max(diff_alt) < torch.max(diff):
            print(f"  Using alternative formula (matches PyTorch better)")
            h_next = h_next_alt
        
        # GRU output is the hidden state (for single-layer GRU)
        x = h_next  # [batch, hidden_size] - this is the GRU output
        
        # Compare with PyTorch GRU output for debugging
        # x2_squeezed is already defined above
        
        # Check for differences
        diff = torch.abs(x - x2_squeezed)
        max_diff = torch.max(diff).item()
        mean_diff = torch.mean(diff).item()
        
        if max_diff > 1e-5:  # Significant difference threshold
            print(f"WARNING: Significant difference between explicit and PyTorch GRU!")
            print(f"  Max difference: {max_diff:.2e}")
            print(f"  Mean difference: {mean_diff:.2e}")
            print(f"  First 10 elements of explicit: {x[0, :10]}")
            print(f"  First 10 elements of PyTorch:  {x2_squeezed[0, :10]}")
            print(f"  First 10 differences:          {diff[0, :10]}")
            
            # Debug: Check intermediate values
            print(f"\n  Debugging intermediate values:")
            print(f"  Input shape: {x_input.shape}, Hidden prev shape: {h_prev.shape}")
            print(f"  z_t first 5: {z_t[0, :5]}, z_h first 5: {z_h[0, :5]}")
            print(f"  r_t first 5: {r_t[0, :5]}, r_h first 5: {r_h[0, :5]}")
            print(f"  n_t first 5: {n_t[0, :5]}, n_h first 5: {n_h[0, :5]}")
            print(f"  z gate first 5: {z[0, :5]}")
            print(f"  r gate first 5: {r[0, :5]}")
            print(f"  n gate first 5: {n[0, :5]}")
            print(f"  h_prev first 5: {h_prev[0, :5]}")
            print(f"  h_next first 5: {h_next[0, :5]}")
            print(f"  x2_squeezed first 5: {x2_squeezed[0, :5]}")
            
            # Check if weights match
            print(f"\n  Weight shapes check:")
            print(f"  W_z shape: {W_z.shape}, W_r shape: {W_r.shape}, W_n shape: {W_n.shape}")
            print(f"  U_z shape: {U_z.shape}, U_r shape: {U_r.shape}, U_n shape: {U_n.shape}")
            print(f"  Bias shapes: b_z_ih {b_z_ih.shape}, b_z_hh {b_z_hh.shape}")
        
        # Keep internal hidden state for next call (needs [1, batch, hidden_size] format)
        self.hxs = h_next.unsqueeze(0)  # [1, batch, hidden_size]

        # Decoder (identity by default)
        if self.decoder is not None:
            x = self.decoder(x)

        # Distribution parameters [mean, logstd]
        params = self.dist_linear(x)
        mean, logstd = params.chunk(2, dim=-1)
        action_logits = torch.cat([mean, logstd], dim=-1)
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
        """Reset internal RNN state to zeros for the given batch size."""
        if device is None:
            device = next(self.parameters()).device
        self.hxs = torch.zeros(1, batch_size, self.core.hidden_size, device=device)

    def set_hidden_state(self, hxs: torch.Tensor) -> None:
        """Set internal RNN state to match external state.
        
        Args:
            hxs: Hidden state tensor of shape [batch, hidden_size] or [1, batch, hidden_size]
        """
        if hxs.dim() == 2:
            # Convert [batch, hidden_size] to [1, batch, hidden_size]
            self.hxs = hxs.unsqueeze(0).clone()
        else:
            # Already [1, batch, hidden_size] or [num_layers, batch, hidden_size]
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
        """Find ordered Linear layer indices inside encoder.encoders.obs.mlp_head.*
        Sample Factory create_mlp uses indices 0,2,4,... for Linear layers (activations in between).
        """
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
            # No MLP layers configured
            enc = nn.Identity()
            return enc

        # Construct layers in the exact order of indices
        layers: list[nn.Module] = []
        for i, idx in enumerate(indices):
            w_key = f'encoder.encoders.obs.mlp_head.{idx}.weight'
            b_key = f'encoder.encoders.obs.mlp_head.{idx}.bias'
            weight = ckpt[w_key]
            bias = ckpt[b_key]
            in_f, out_f = weight.shape[1], weight.shape[0]
            lin = nn.Linear(in_f, out_f)
            # Load weights immediately to avoid dtype/device surprises later
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

        # Observation normalizer parameters (RunningMeanStd)
        mean_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_mean'
        var_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_var'
        if mean_key in ckpt and var_key in ckpt:
            obs_mean = ckpt[mean_key].detach()
            obs_var = ckpt[var_key].detach()
        else:
            # Fallback: infer input size from the first linear layer if present, otherwise 1
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

        # Build encoder dynamically from checkpoint
        encoder = cls._build_encoder_from_ckpt(ckpt, nonlinearity=nonlinearity, jit=jit_encoder)

        # Determine GRU input size from the last linear layer out_features or keep 512 default
        gru_input_size = 512
        lin_indices = cls._find_mlp_linear_indices(ckpt)
        if len(lin_indices) > 0:
            last_idx = lin_indices[-1]
            last_w = ckpt[f'encoder.encoders.obs.mlp_head.{last_idx}.weight']
            gru_input_size = last_w.shape[0]

        # GRU core (hidden size inferred from checkpoint)
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


if __name__ == "__main__":
    import argparse
    import torch

    parser = argparse.ArgumentParser(description="RLPolicyClean one-step inference example")
    parser.add_argument("--ckpt", type=str, required=True, help="Path to Sample Factory checkpoint .pth")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Inference device")
    parser.add_argument("--nonlinearity", type=str, default="relu", choices=["relu", "elu", "tanh"], help="Activation to use (must match training)")
    parser.add_argument("--jit_encoder", action="store_true", help="Enable torch.jit.script for encoder MLP")
    parser.add_argument("--normalized", action="store_true", help="Treat provided obs as already normalized")
    args = parser.parse_args()

    # Load policy
    policy = RLPolicyClean.load_from_checkpoint(
        args.ckpt,
        device=args.device,
        nonlinearity=args.nonlinearity,
        jit_encoder=args.jit_encoder,
    ).eval()

    # Prepare a dummy observation (batch=1) matching checkpoint input size
    # If not normalized, obs will be normalized internally using checkpoint stats
    with torch.no_grad():
        in_size = policy.obs_mean.shape[0]
        obs = torch.zeros(1, in_size, device=args.device)
        policy.reset_hidden_state(batch_size=1, device=torch.device(args.device))

        action_logits = policy(obs, normalized=args.normalized)
        print("action_logits shape:", tuple(action_logits.shape))
        print("hidden state shape:", tuple(policy.hxs.shape))
        print("action_logits (first row):", action_logits[0].tolist())

