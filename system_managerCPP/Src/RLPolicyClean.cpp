#include "RLPolicyClean.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <memory>

RLPolicyClean::RLPolicyClean() : hxs(), gru_hidden_size(512) {
}


std::vector<int> RLPolicyClean::_find_mlp_linear_indices(const std::string& checkpoint_path) {
    // This would need to load the checkpoint and parse keys
    // For now, return empty - actual implementation would load torch checkpoint
    std::vector<int> indices;
    // TODO: Implement checkpoint key parsing
    return indices;
}

std::shared_ptr<torch::nn::Sequential> RLPolicyClean::_build_encoder_from_ckpt(
    const std::string& checkpoint_path,
    const std::string& nonlinearity,
    bool jit) {
    
    // Placeholder - would load from checkpoint
    // In practice, this would:
    // 1. Load checkpoint
    // 2. Find MLP layer indices
    // 3. Build Sequential module with Linear/Activation layers
    
    // Build Sequential module - Sequential is a ModuleHolder, use -> to access methods
    torch::nn::Sequential seq;
    seq->push_back("linear1", torch::nn::Linear(torch::nn::LinearOptions(4, 64)));
    if (nonlinearity == "relu") {
        seq->push_back("act1", torch::nn::ReLU(torch::nn::ReLUOptions().inplace(false)));
    } else if (nonlinearity == "elu") {
        seq->push_back("act1", torch::nn::ELU(torch::nn::ELUOptions().inplace(false)));
    } else if (nonlinearity == "tanh") {
        seq->push_back("act1", torch::nn::Tanh());
    }
    
    seq->push_back("linear2", torch::nn::Linear(torch::nn::LinearOptions(64, 128)));
    if (nonlinearity == "relu") {
        seq->push_back("act2", torch::nn::ReLU(torch::nn::ReLUOptions().inplace(false)));
    } else if (nonlinearity == "elu") {
        seq->push_back("act2", torch::nn::ELU(torch::nn::ELUOptions().inplace(false)));
    } else if (nonlinearity == "tanh") {
        seq->push_back("act2", torch::nn::Tanh());
    }
    
    seq->push_back("linear3", torch::nn::Linear(torch::nn::LinearOptions(128, 256)));
    if (nonlinearity == "relu") {
        seq->push_back("act3", torch::nn::ReLU(torch::nn::ReLUOptions().inplace(false)));
    } else if (nonlinearity == "elu") {
        seq->push_back("act3", torch::nn::ELU(torch::nn::ELUOptions().inplace(false)));
    } else if (nonlinearity == "tanh") {
        seq->push_back("act3", torch::nn::Tanh());
    }
    
    // Return as shared_ptr
    return std::make_shared<torch::nn::Sequential>(seq);
}

void RLPolicyClean::_init_modules(
    const VectorXd& obs_mean,
    const VectorXd& obs_var,
    std::shared_ptr<torch::nn::Sequential> encoder,
    std::shared_ptr<torch::nn::GRU> core,
    std::shared_ptr<torch::nn::Linear> dist_linear,
    std::shared_ptr<torch::nn::Sequential> decoder,
    int gru_hidden_size) {

    this->obs_mean = obs_mean;
    this->obs_var = obs_var;
    this->encoder = encoder;
    this->core = core;
    this->dist_linear = dist_linear;
    this->decoder = decoder;
    this->hxs = torch::Tensor();
    this->gru_hidden_size = gru_hidden_size;
}

std::pair<VectorXd, VectorXd> RLPolicyClean::forward(
    const VectorXd& obs,
    bool normalized) {
    
    if (!encoder || !core || !dist_linear) {
        throw std::runtime_error("RLPolicyClean not properly initialized");
    }
    
    // Convert Eigen to torch tensor
    std::vector<double> obs_vec(obs.data(), obs.data() + obs.size());
    torch::Tensor obs_tensor = torch::from_blob(
        obs_vec.data(),
        {1, static_cast<long>(obs.size())},
        torch::kFloat64
    ).clone().to(torch::kFloat32);
    
    // Normalize if needed
    if (!normalized) {
        std::vector<double> mean_vec(obs_mean.data(), obs_mean.data() + obs_mean.size());
        std::vector<double> var_vec(obs_var.data(), obs_var.data() + obs_var.size());
        
        torch::Tensor mean_tensor = torch::from_blob(
            mean_vec.data(),
            {static_cast<long>(obs_mean.size())},
            torch::kFloat64
        ).clone().to(torch::kFloat32);
        
        torch::Tensor var_tensor = torch::from_blob(
            var_vec.data(),
            {static_cast<long>(obs_var.size())},
            torch::kFloat64
        ).clone().to(torch::kFloat32);
        
        obs_tensor = (obs_tensor - mean_tensor) / torch::sqrt(var_tensor + EPS);
        obs_tensor = torch::clamp(obs_tensor, -5.0, 5.0);
    }
    
    // Encoder - Sequential is a ModuleHolder, access underlying impl with (*encoder)->
    torch::Tensor x = (*encoder)->forward(obs_tensor);
    
    // GRU expects [seq_len, batch, features]
    x = x.unsqueeze(0);  // [batch, features] -> [1, batch, features]
    
    // Initialize hidden state if needed
    if (!hxs.defined() || hxs.size(1) != x.size(1)) {
        hxs = torch::zeros({1, x.size(1), gru_hidden_size}, torch::kFloat32);
    }
    
    // GRU forward - In LibTorch C++, RNN modules use operator() with input
    // For hidden state, we need to use the module's internal state or pass it differently
    // GRU's operator() signature: operator()(input, hidden) -> tuple<output, hidden>
    auto gru_out = (*core)(x, hxs);
    x = std::get<0>(gru_out);
    torch::Tensor h_next = std::get<1>(gru_out);
    x = x.squeeze(0);  // [1, batch, hidden] -> [batch, hidden]
    
    // Update hidden state
    hxs = h_next;
    
    // Decoder (if present)
    if (decoder) {
        x = (*decoder)->forward(x);
    }
    
    // Distribution parameters - Linear uses operator()
    torch::Tensor params = (*dist_linear)(x);
    int action_dim = params.size(1) / 2;
    torch::Tensor mean = params.slice(1, 0, action_dim);
    torch::Tensor logstd = params.slice(1, action_dim);
    torch::Tensor action_logits = torch::cat({mean, logstd}, 1);
    
    // Convert back to Eigen
    VectorXd action_logits_eigen(action_logits.size(1));
    auto action_logits_cpu = action_logits.cpu();
    auto action_logits_accessor = action_logits_cpu.accessor<float, 2>();
    for (int i = 0; i < action_logits.size(1); i++) {
        action_logits_eigen[i] = static_cast<double>(action_logits_accessor[0][i]);
    }
    
    VectorXd h_next_eigen(h_next.size(2));
    auto h_next_cpu = h_next.cpu();
    auto h_next_accessor = h_next_cpu.accessor<float, 3>();
    for (int i = 0; i < h_next.size(2); i++) {
        h_next_eigen[i] = static_cast<double>(h_next_accessor[0][0][i]);
    }
    
    return std::make_pair(action_logits_eigen, h_next_eigen);
}

void RLPolicyClean::reset_hidden_state(int batch_size) {
    if (core) {
        hxs = torch::zeros({1, batch_size, gru_hidden_size}, torch::kFloat32);
    }
}

void RLPolicyClean::set_hidden_state(const VectorXd& hxs_vec) {
    std::vector<double> hxs_vec_data(hxs_vec.data(), hxs_vec.data() + hxs_vec.size());
    torch::Tensor hxs_tensor = torch::from_blob(
        hxs_vec_data.data(),
        {1, 1, static_cast<long>(hxs_vec.size())},
        torch::kFloat64
    ).clone().to(torch::kFloat32);
    hxs = hxs_tensor;
}

std::shared_ptr<RLPolicyClean> RLPolicyClean::load_from_checkpoint(
    const std::string& path,
    const std::string& device,
    const std::string& nonlinearity,
    bool jit_encoder) {
    
    auto policy = std::make_shared<RLPolicyClean>();
    
    try {
        // Load checkpoint - PyTorch checkpoints are typically saved as IValue dictionaries
        torch::Device torch_device(device == "cuda" ? torch::kCUDA : torch::kCPU);
        
        // Load checkpoint as IValue using InputArchive
        // PyTorch checkpoints are saved as dictionaries, typically at root level
        torch::IValue checkpoint_ivalue;
        bool checkpoint_loaded = false;
        try {
            torch::serialize::InputArchive archive;
            archive.load_from(path);
            // Try reading with common root keys used in PyTorch checkpoints
            // First try empty key (root), then try "model" key
            try {
                archive.read("", checkpoint_ivalue);
                checkpoint_loaded = true;
            } catch (const std::exception& e1) {
                try {
                    archive.read("model", checkpoint_ivalue);
                    checkpoint_loaded = true;
                } catch (const std::exception& e2) {
                    // If both fail, try to read as a generic IValue
                    // This might work for some checkpoint formats
                    std::cerr << "Warning: Could not read checkpoint with empty or 'model' key" << std::endl;
                    checkpoint_loaded = false;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not load checkpoint file: " << e.what() << std::endl;
            std::cerr << "Will use default policy structure" << std::endl;
            checkpoint_loaded = false;
        }
        
        // Extract observation normalization parameters
        torch::Tensor obs_mean_tensor;
        torch::Tensor obs_var_tensor;
        
        // Try to find keys in checkpoint
        bool found_mean = false;
        bool found_var = false;
        
        if (checkpoint_loaded && checkpoint_ivalue.isGenericDict()) {
            // Search for keys (they might have slightly different paths)
            auto checkpoint_dict = checkpoint_ivalue.toGenericDict();
            for (const auto& pair : checkpoint_dict) {
                std::string key = pair.key().toStringRef();
                auto value = pair.value();
                
                if (value.isTensor()) {
                    if (key.find("running_mean") != std::string::npos && key.find("obs") != std::string::npos) {
                        obs_mean_tensor = value.toTensor().cpu();
                        found_mean = true;
                    }
                    if (key.find("running_var") != std::string::npos && key.find("obs") != std::string::npos) {
                        obs_var_tensor = value.toTensor().cpu();
                        found_var = true;
                    }
                } else if (value.isGenericDict()) {
                    // Check nested dictionaries (e.g., "model" key)
                    auto nested_dict = value.toGenericDict();
                    for (const auto& nested_pair : nested_dict) {
                        std::string nested_key = nested_pair.key().toStringRef();
                        if (nested_pair.value().isTensor()) {
                            if (nested_key.find("running_mean") != std::string::npos && nested_key.find("obs") != std::string::npos) {
                                obs_mean_tensor = nested_pair.value().toTensor().cpu();
                                found_mean = true;
                            }
                            if (nested_key.find("running_var") != std::string::npos && nested_key.find("obs") != std::string::npos) {
                                obs_var_tensor = nested_pair.value().toTensor().cpu();
                                found_var = true;
                            }
                        }
                    }
                }
            }
        }
        
        int obs_size = 4;  // Default
        VectorXd obs_mean_vec;
        VectorXd obs_var_vec;
        
        if (found_mean && found_var) {
            obs_size = obs_mean_tensor.size(0);
            obs_mean_vec = VectorXd(obs_size);
            obs_var_vec = VectorXd(obs_size);
            
            auto mean_accessor = obs_mean_tensor.accessor<float, 1>();
            auto var_accessor = obs_var_tensor.accessor<float, 1>();
            for (int i = 0; i < obs_size; i++) {
                obs_mean_vec[i] = static_cast<double>(mean_accessor[i]);
                obs_var_vec[i] = static_cast<double>(var_accessor[i]);
            }
        } else {
            // Fallback: use zeros and ones
            obs_mean_vec = VectorXd::Zero(obs_size);
            obs_var_vec = VectorXd::Ones(obs_size);
        }
        
        // Build encoder (simplified - would parse checkpoint keys)
        auto encoder_module = _build_encoder_from_ckpt(path, nonlinearity, jit_encoder);
        
        // Build GRU (would load weights from checkpoint)
        int gru_input_size = 256;  // Would be determined from encoder output
        int gru_hidden_size = 512;  // Would be loaded from checkpoint
        auto core_module = std::make_shared<torch::nn::GRU>(
            torch::nn::GRUOptions(gru_input_size, gru_hidden_size).batch_first(false));
        
        // Load GRU weights if available
        // This would require parsing checkpoint keys like 'core.core.weight_ih_l0', etc.
        
        // Build distribution head
        int dist_in = gru_hidden_size;
        int num_actions = 2;  // Would be determined from checkpoint
        auto dist_linear_module = std::make_shared<torch::nn::Linear>(
            torch::nn::LinearOptions(dist_in, num_actions * 2));  // mean + logstd for each action
        
        // Load distribution weights if available
        // This would require parsing 'action_parameterization.distribution_linear.*' keys
        
        policy->_init_modules(obs_mean_vec, obs_var_vec, encoder_module, 
                            core_module, dist_linear_module, nullptr, gru_hidden_size);
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading checkpoint: " << e.what() << std::endl;
        std::cerr << "Using default policy structure" << std::endl;
        
        // Fallback: create minimal policy
        int obs_size = 4;
        VectorXd obs_mean_vec = VectorXd::Zero(obs_size);
        VectorXd obs_var_vec = VectorXd::Ones(obs_size);
        
        auto encoder_module = _build_encoder_from_ckpt(path, nonlinearity, jit_encoder);
        int gru_hidden_size_fallback = 512;
        auto core_module = std::make_shared<torch::nn::GRU>(
            torch::nn::GRUOptions(256, gru_hidden_size_fallback).batch_first(false));
        auto dist_linear_module = std::make_shared<torch::nn::Linear>(
            torch::nn::LinearOptions(gru_hidden_size_fallback, 4));
        
        policy->_init_modules(obs_mean_vec, obs_var_vec, encoder_module, 
                            core_module, dist_linear_module, nullptr, gru_hidden_size_fallback);
    }
    
    return policy;
}

