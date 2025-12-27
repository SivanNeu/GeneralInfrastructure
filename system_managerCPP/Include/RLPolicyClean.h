#ifndef RL_POLICY_CLEAN_H
#define RL_POLICY_CLEAN_H
#include "general.h"

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>

// Forward declarations for LibTorch types
// Note: Requires LibTorch to be linked
#include <torch/torch.h>

class RLPolicyClean {
private:
    // Observation normalization parameters
    VectorXd obs_mean;
    VectorXd obs_var;
    
    // Neural network components (LibTorch modules)
    // Sequential cannot be stored in AnyModule (forward() is templated)
    // So we use shared_ptr<Sequential> for encoder/decoder
    std::shared_ptr<torch::nn::Sequential> encoder;
    std::shared_ptr<torch::nn::GRU> core;
    std::shared_ptr<torch::nn::Sequential> decoder;
    std::shared_ptr<torch::nn::Linear> dist_linear;
    
    // Internal recurrent state
    torch::Tensor hxs;
    
    // Store GRU hidden size for initialization
    int gru_hidden_size;
    
    // Constants
    static constexpr double EPS = 1e-5;
    
    // Helper methods
    static std::shared_ptr<torch::nn::Sequential> _build_encoder_from_ckpt(
        const std::string& checkpoint_path,
        const std::string& nonlinearity,
        bool jit);
    static std::vector<int> _find_mlp_linear_indices(const std::string& checkpoint_path);
    
    void _init_modules(
        const VectorXd& obs_mean,
        const VectorXd& obs_var,
        std::shared_ptr<torch::nn::Sequential> encoder,
        std::shared_ptr<torch::nn::GRU> core,
        std::shared_ptr<torch::nn::Linear> dist_linear,
        std::shared_ptr<torch::nn::Sequential> decoder = nullptr,
        int gru_hidden_size = 512);

public:
    RLPolicyClean();
    
    // Forward pass: returns (action_logits, new_hxs)
    // action_logits contains [mean, logstd] concatenated
    std::pair<VectorXd, VectorXd> forward(
        const VectorXd& obs,
        bool normalized = false);
    
    // Reset hidden state
    void reset_hidden_state(int batch_size = 1);
    
    // Set hidden state
    void set_hidden_state(const VectorXd& hxs_vec);
    
    // Load policy from checkpoint
    static std::shared_ptr<RLPolicyClean> load_from_checkpoint(
        const std::string& path,
        const std::string& device = "cpu",
        const std::string& nonlinearity = "relu",
        bool jit_encoder = false);
    
    // Get observation size
    int get_obs_size() const { return static_cast<int>(obs_mean.size()); }
};

#endif // RL_POLICY_CLEAN_H

