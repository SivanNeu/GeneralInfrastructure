/**
 * test_comparison.cpp
 * 
 * Purpose: Compare inference results between two implementations:
 *   1. Reference: libtorch-based implementation loading .pth file directly
 *   2. Test: Manual implementation loading JSON file (converted from .pth)
 * 
 * This test verifies that the manual JSON-based implementation produces
 * the same results as the libtorch reference implementation, ensuring
 * correctness of the manual neural network computation.
 */

#include "RLPolicyJSON.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <set>
#include <limits>

// Reference implementation: libtorch-based policy loader (mirrors Python RLPolicyClean)
class RLPolicyTorch {
private:
    torch::Tensor obs_mean_;
    torch::Tensor obs_var_;
    torch::nn::Sequential encoder_;
    torch::nn::GRU core_;
    torch::nn::Linear dist_linear_;
    torch::Tensor hxs_;
    
    static constexpr float EPS = 1e-5f;
    
    static std::vector<int> find_mlp_linear_indices(const c10::Dict<at::IValue, at::IValue>& ckpt) {
        std::vector<int> indices;
        std::set<int> index_set;
        std::string prefix = "encoder.encoders.obs.mlp_head.";
        
        for (const auto& pair : ckpt) {
            std::string key = pair.key().toStringRef();
            if (key.find(prefix) == 0 && key.find(".weight") != std::string::npos) {
                size_t idx_start = prefix.length();
                size_t idx_end = key.find('.', idx_start);
                if (idx_end != std::string::npos) {
                    try {
                        int idx = std::stoi(key.substr(idx_start, idx_end - idx_start));
                        index_set.insert(idx);
                    } catch (...) {
                        continue;
                    }
                }
            }
        }
        
        indices.assign(index_set.begin(), index_set.end());
        std::sort(indices.begin(), indices.end());
        return indices;
    }
    
    static torch::nn::Sequential build_encoder_from_ckpt(
        const c10::Dict<at::IValue, at::IValue>& ckpt,
        const std::string& nonlinearity) {
        
        torch::nn::Sequential encoder;
        std::vector<int> indices = find_mlp_linear_indices(ckpt);
        
        if (indices.empty()) {
            return encoder;  // Identity
        }
        
        for (int idx : indices) {
            std::string w_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".weight";
            std::string b_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".bias";
            
            auto w_tensor = ckpt.at(w_key).toTensor();
            auto b_tensor = ckpt.at(b_key).toTensor();
            
            int in_features = w_tensor.size(1);
            int out_features = w_tensor.size(0);
            
            torch::nn::Linear linear(torch::nn::LinearOptions(in_features, out_features));
            linear->weight.data().copy_(w_tensor);
            linear->bias.data().copy_(b_tensor);
            
            encoder->push_back("linear" + std::to_string(idx), linear);
            
            // Create activation module for each layer
            if (nonlinearity == "relu") {
                encoder->push_back("act" + std::to_string(idx), 
                    torch::nn::ReLU(torch::nn::ReLUOptions().inplace(false)));
            } else if (nonlinearity == "elu") {
                encoder->push_back("act" + std::to_string(idx), 
                    torch::nn::ELU(torch::nn::ELUOptions().inplace(false)));
            } else if (nonlinearity == "tanh") {
                encoder->push_back("act" + std::to_string(idx), torch::nn::Tanh());
            }
        }
        
        return encoder;
    }
    
public:
    static std::shared_ptr<RLPolicyTorch> load_from_checkpoint(
        const std::string& path,
        const std::string& device = "cpu",
        const std::string& nonlinearity = "relu") {
        
        auto policy = std::make_shared<RLPolicyTorch>();
        
        // Load checkpoint
        torch::Device torch_device(device == "cuda" ? torch::kCUDA : torch::kCPU);
        auto checkpoint = torch::load(path, torch_device);
        
        // Extract model dict
        c10::Dict<at::IValue, at::IValue> model_dict;
        if (checkpoint.isGenericDict()) {
            auto dict = checkpoint.toGenericDict();
            if (dict.contains("model")) {
                model_dict = dict.at("model").toGenericDict();
            } else {
                model_dict = dict;
            }
        } else {
            throw std::runtime_error("Checkpoint format not recognized");
        }
        
        // Load normalizer
        std::string mean_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_mean";
        std::string var_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_var";
        
        if (model_dict.contains(mean_key) && model_dict.contains(var_key)) {
            policy->obs_mean_ = model_dict.at(mean_key).toTensor().to(torch::kFloat32);
            policy->obs_var_ = model_dict.at(var_key).toTensor().to(torch::kFloat32);
        } else {
            // Fallback
            std::vector<int> indices = find_mlp_linear_indices(model_dict);
            if (!indices.empty()) {
                std::string first_w_key = "encoder.encoders.obs.mlp_head." + std::to_string(indices[0]) + ".weight";
                auto first_w = model_dict.at(first_w_key).toTensor();
                int in_size = first_w.size(1);
                policy->obs_mean_ = torch::zeros({in_size}, torch::kFloat32);
                policy->obs_var_ = torch::ones({in_size}, torch::kFloat32);
            } else {
                policy->obs_mean_ = torch::zeros({1}, torch::kFloat32);
                policy->obs_var_ = torch::ones({1}, torch::kFloat32);
            }
        }
        
        // Build encoder
        policy->encoder_ = build_encoder_from_ckpt(model_dict, nonlinearity);
        
        // Determine GRU input size
        std::vector<int> indices = find_mlp_linear_indices(model_dict);
        int gru_input_size = 512;
        if (!indices.empty()) {
            std::string last_w_key = "encoder.encoders.obs.mlp_head." + std::to_string(indices.back()) + ".weight";
            auto last_w = model_dict.at(last_w_key).toTensor();
            gru_input_size = last_w.size(0);
        }
        
        // Load GRU
        auto weight_hh = model_dict.at("core.core.weight_hh_l0").toTensor();
        int rnn_size = weight_hh.size(1);
        policy->core_ = torch::nn::GRU(torch::nn::GRUOptions(gru_input_size, rnn_size).batch_first(false));
        policy->core_->weight_ih_l0.data().copy_(model_dict.at("core.core.weight_ih_l0").toTensor());
        policy->core_->weight_hh_l0.data().copy_(weight_hh);
        policy->core_->bias_ih_l0.data().copy_(model_dict.at("core.core.bias_ih_l0").toTensor());
        policy->core_->bias_hh_l0.data().copy_(model_dict.at("core.core.bias_hh_l0").toTensor());
        
        // Load distribution linear
        auto dist_w = model_dict.at("action_parameterization.distribution_linear.weight").toTensor();
        auto dist_b = model_dict.at("action_parameterization.distribution_linear.bias").toTensor();
        int dist_in = dist_w.size(1);
        int num_actions = dist_w.size(0);
        policy->dist_linear_ = torch::nn::Linear(torch::nn::LinearOptions(dist_in, num_actions));
        policy->dist_linear_->weight.data().copy_(dist_w);
        policy->dist_linear_->bias.data().copy_(dist_b);
        
        policy->reset_hidden_state(1);
        return policy;
    }
    
    void reset_hidden_state(int batch_size = 1) {
        int hidden_size = core_->options.hidden_size();
        hxs_ = torch::zeros({1, batch_size, hidden_size}, torch::kFloat32);
    }
    
    std::pair<std::vector<float>, std::vector<float>> forward(
        const std::vector<float>& obs,
        bool normalized = false) {
        
        // Convert to tensor
        torch::Tensor obs_tensor = torch::from_blob(
            const_cast<float*>(obs.data()),
            {1, static_cast<long>(obs.size())},
            torch::kFloat32
        ).clone();
        
        // Normalize
        if (!normalized) {
            obs_tensor = (obs_tensor - obs_mean_) / torch::sqrt(obs_var_ + EPS);
            obs_tensor = torch::clamp(obs_tensor, -5.0f, 5.0f);
        }
        
        // Encoder
        torch::Tensor x = encoder_->forward(obs_tensor);
        
        // GRU
        x = x.unsqueeze(0);  // [batch, features] -> [1, batch, features]
        auto gru_out = core_->forward(x, hxs_);
        x = std::get<0>(gru_out);
        hxs_ = std::get<1>(gru_out);
        x = x.squeeze(0);  // [1, batch, hidden] -> [batch, hidden]
        
        // Distribution
        torch::Tensor params = dist_linear_->forward(x);
        int action_dim = params.size(1) / 2;
        torch::Tensor mean = params.slice(1, 0, action_dim);
        torch::Tensor logstd = params.slice(1, action_dim);
        torch::Tensor action_logits = torch::cat({mean, logstd}, 1);
        
        // Convert to vectors
        std::vector<float> action_vec(action_logits.size(1));
        auto action_cpu = action_logits.cpu();
        auto accessor = action_cpu.accessor<float, 2>();
        for (int i = 0; i < action_logits.size(1); i++) {
            action_vec[i] = accessor[0][i];
        }
        
        std::vector<float> hxs_vec(hxs_.size(2));
        auto hxs_cpu = hxs_.cpu();
        auto hxs_accessor = hxs_cpu.accessor<float, 3>();
        for (int i = 0; i < hxs_.size(2); i++) {
            hxs_vec[i] = hxs_accessor[0][0][i];
        }
        
        return std::make_pair(action_vec, hxs_vec);
    }
    
    int get_obs_size() const {
        return obs_mean_.size(0);
    }
    
    int get_action_size() const {
        return dist_linear_->options.out_features();
    }
};

float compute_max_diff(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) {
        return std::numeric_limits<float>::infinity();
    }
    
    float max_diff = 0.0f;
    for (size_t i = 0; i < a.size(); i++) {
        float diff = std::abs(a[i] - b[i]);
        max_diff = std::max(max_diff, diff);
    }
    return max_diff;
}

float compute_relative_error(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) {
        return std::numeric_limits<float>::infinity();
    }
    
    float max_rel_error = 0.0f;
    for (size_t i = 0; i < a.size(); i++) {
        float abs_val = std::max(std::abs(a[i]), std::abs(b[i]));
        if (abs_val > 1e-8f) {
            float rel_error = std::abs(a[i] - b[i]) / abs_val;
            max_rel_error = std::max(max_rel_error, rel_error);
        }
    }
    return max_rel_error;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <pth_file> <json_file> [nonlinearity] [normalized]" << std::endl;
        std::cerr << "\nPurpose: Compare inference results between:" << std::endl;
        std::cerr << "  - Reference: libtorch loading .pth file directly" << std::endl;
        std::cerr << "  - Test: Manual implementation loading JSON file" << std::endl;
        std::cerr << "\nArguments:" << std::endl;
        std::cerr << "  pth_file: Path to PyTorch checkpoint (.pth) - used by libtorch reference" << std::endl;
        std::cerr << "  json_file: Path to JSON file (converted from .pth) - used by manual implementation" << std::endl;
        std::cerr << "  nonlinearity: relu|elu|tanh (default: relu) - must match training config" << std::endl;
        std::cerr << "  normalized: 0|1 (default: 0) - whether input is already normalized" << std::endl;
        return 1;
    }
    
    std::string pth_path = argv[1];
    std::string json_path = argv[2];
    std::string nonlinearity_str = (argc > 3) ? argv[3] : "relu";
    bool normalized = (argc > 4) ? (std::stoi(argv[4]) != 0) : false;
    
    RLPolicyJSON::Activation activation = RLPolicyJSON::Activation::RELU;
    if (nonlinearity_str == "elu") {
        activation = RLPolicyJSON::Activation::ELU;
    } else if (nonlinearity_str == "tanh") {
        activation = RLPolicyJSON::Activation::TANH;
    }
    
    std::cout << "=== Test Comparison: libtorch (reference) vs JSON (manual) ===" << std::endl;
    std::cout << "\n[1/2] Loading manual JSON-based policy from: " << json_path << std::endl;
    RLPolicyJSON policy_json;
    if (!policy_json.load_from_json(json_path, activation)) {
        std::cerr << "Error: Failed to load JSON policy" << std::endl;
        return 1;
    }
    std::cout << "  ✓ JSON policy loaded successfully" << std::endl;
    
    std::cout << "\n[2/2] Loading libtorch reference policy from: " << pth_path << std::endl;
    std::shared_ptr<RLPolicyTorch> policy_torch;
    try {
        policy_torch = RLPolicyTorch::load_from_checkpoint(pth_path, "cpu", nonlinearity_str);
    } catch (const std::exception& e) {
        std::cerr << "Error: Failed to load libtorch policy: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "  ✓ libtorch policy loaded successfully" << std::endl;
    
    // Verify sizes match
    if (policy_json.get_obs_size() != policy_torch->get_obs_size()) {
        std::cerr << "Error: Observation size mismatch!" << std::endl;
        std::cerr << "  JSON: " << policy_json.get_obs_size() << ", libtorch: " << policy_torch->get_obs_size() << std::endl;
        return 1;
    }
    
    if (policy_json.get_action_size() != policy_torch->get_action_size()) {
        std::cerr << "Error: Action size mismatch!" << std::endl;
        std::cerr << "  JSON: " << policy_json.get_action_size() << ", libtorch: " << policy_torch->get_action_size() << std::endl;
        return 1;
    }
    
    std::cout << "\nPolicy configurations match:" << std::endl;
    std::cout << "  Observation size: " << policy_json.get_obs_size() << std::endl;
    std::cout << "  Action size: " << policy_json.get_action_size() << std::endl;
    std::cout << "  Nonlinearity: " << nonlinearity_str << std::endl;
    std::cout << "  Normalized input: " << (normalized ? "yes" : "no") << std::endl;
    
    // Reset hidden states
    policy_json.reset_hidden_state(1);
    policy_torch->reset_hidden_state(1);
    
    // Create test observation (zeros)
    std::vector<float> obs(policy_json.get_obs_size(), 0.0f);
    
    // Forward pass - run both implementations
    std::cout << "\n=== Running Forward Pass ===" << std::endl;
    std::cout << "Running manual JSON implementation..." << std::endl;
    std::vector<float> action_json = policy_json.forward(obs, normalized);
    std::cout << "Running libtorch reference implementation..." << std::endl;
    auto [action_torch, hxs_torch] = policy_torch->forward(obs, normalized);
    std::vector<float> hxs_json = policy_json.get_hidden_state();
    
    // Compare results
    std::cout << "\n=== Comparison Results ===" << std::endl;
    std::cout << "Comparing outputs from both implementations..." << std::endl;
    
    float action_max_diff = compute_max_diff(action_json, action_torch);
    float action_rel_error = compute_relative_error(action_json, action_torch);
    
    std::cout << "Action logits:" << std::endl;
    std::cout << "  Max absolute difference: " << std::scientific << action_max_diff << std::endl;
    std::cout << "  Max relative error: " << std::scientific << action_rel_error << std::endl;
    
    float hxs_max_diff = compute_max_diff(hxs_json, hxs_torch);
    float hxs_rel_error = compute_relative_error(hxs_json, hxs_torch);
    
    std::cout << "Hidden state:" << std::endl;
    std::cout << "  Max absolute difference: " << std::scientific << hxs_max_diff << std::endl;
    std::cout << "  Max relative error: " << std::scientific << hxs_rel_error << std::endl;
    
    // Print first few values for inspection
    std::cout << "\nSample values (first 5 action logits):" << std::endl;
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "  Index | JSON (manual) | libtorch (ref) | Difference" << std::endl;
    std::cout << "  ----- | ------------ | -------------- | ----------" << std::endl;
    for (int i = 0; i < std::min(5, (int)action_json.size()); i++) {
        float diff = std::abs(action_json[i] - action_torch[i]);
        std::cout << "  [" << std::setw(3) << i << "] | " 
                  << std::setw(12) << action_json[i] << " | "
                  << std::setw(14) << action_torch[i] << " | "
                  << std::setw(10) << diff << std::endl;
    }
    if (action_json.size() > 5) {
        std::cout << "  ... (total " << action_json.size() << " values)" << std::endl;
    }
    
    // Tolerance check
    const float TOLERANCE = 1e-4f;
    bool action_match = action_max_diff < TOLERANCE;
    bool hxs_match = hxs_max_diff < TOLERANCE;
    
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Tolerance threshold: " << std::scientific << TOLERANCE << std::endl;
    std::cout << "\nAction logits:" << std::endl;
    std::cout << "  Max absolute difference: " << std::scientific << action_max_diff << std::endl;
    std::cout << "  Max relative error: " << std::scientific << action_rel_error << std::endl;
    std::cout << "  Match: " << (action_match ? "✓ YES" : "✗ NO") << std::endl;
    
    std::cout << "\nHidden state:" << std::endl;
    std::cout << "  Max absolute difference: " << std::scientific << hxs_max_diff << std::endl;
    std::cout << "  Max relative error: " << std::scientific << hxs_rel_error << std::endl;
    std::cout << "  Match: " << (hxs_match ? "✓ YES" : "✗ NO") << std::endl;
    
    if (action_match && hxs_match) {
        std::cout << "\n✓ SUCCESS: Manual JSON implementation matches libtorch reference!" << std::endl;
        std::cout << "  The JSON-based implementation produces correct results." << std::endl;
        return 0;
    } else {
        std::cout << "\n✗ FAILURE: Implementations differ beyond tolerance!" << std::endl;
        std::cout << "  Please check:" << std::endl;
        std::cout << "    - JSON file conversion correctness" << std::endl;
        std::cout << "    - Manual implementation accuracy" << std::endl;
        std::cout << "    - Numerical precision issues" << std::endl;
        return 1;
    }
}
