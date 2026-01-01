#include "RLPolicyClean.h"
#include "utils/SimpleJSONParser.h"
#include <iostream>
#include <stdexcept>

RLPolicyClean::RLPolicyClean() : gru_hidden_size(512), gru_input_size(0) {
    hxs = VectorXd::Zero(gru_hidden_size);
}

bool RLPolicyClean::load_normalizer_from_json() {
    if (!json_parser_) return false;
    
    std::string mean_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_mean";
    std::string var_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_var";
    
    std::vector<float> mean_data, var_data;
    std::vector<int> mean_shape, var_shape;
    
    if (json_parser_->extract_tensor(mean_key, mean_data, mean_shape) && 
        json_parser_->extract_tensor(var_key, var_data, var_shape)) {
        int obs_size = mean_shape.empty() ? mean_data.size() : mean_shape[0];
        obs_mean = VectorXd(obs_size);
        obs_var = VectorXd(obs_size);
        for (int i = 0; i < obs_size; i++) {
            obs_mean[i] = static_cast<double>(mean_data[i]);
            obs_var[i] = static_cast<double>(var_data[i]);
        }
        return true;
    }
    
    // Fallback: infer from encoder
    std::vector<int> indices;
    if (json_parser_->find_mlp_linear_indices(indices) && !indices.empty()) {
        std::string first_w_key = "encoder.encoders.obs.mlp_head." + std::to_string(indices[0]) + ".weight";
        std::vector<float> first_w_data;
        std::vector<int> first_w_shape;
        if (json_parser_->extract_tensor(first_w_key, first_w_data, first_w_shape) && first_w_shape.size() >= 2) {
            int obs_size = first_w_shape[1];
            obs_mean = VectorXd::Zero(obs_size);
            obs_var = VectorXd::Ones(obs_size);
            return true;
        }
    }
    
    // Last resort
    obs_mean = VectorXd::Zero(1);
    obs_var = VectorXd::Ones(1);
    return true;
}

bool RLPolicyClean::build_encoder_from_json(Activation nonlinearity) {
    if (!json_parser_) return false;
    
    encoder_layers_.clear();
    
    std::vector<int> indices;
    if (!json_parser_->find_mlp_linear_indices(indices)) {
        return true;  // No encoder layers - use identity
    }
    
    for (int idx : indices) {
        std::string w_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".weight";
        std::string b_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".bias";
        
        std::vector<float> w_data, b_data;
        std::vector<int> w_shape, b_shape;
        
        if (!json_parser_->extract_tensor(w_key, w_data, w_shape) || 
            !json_parser_->extract_tensor(b_key, b_data, b_shape)) {
            std::cerr << "Warning: Could not load encoder layer " << idx << std::endl;
            continue;
        }
        
        if (w_shape.size() != 2) {
            std::cerr << "Warning: Invalid weight shape for layer " << idx << std::endl;
            continue;
        }
        
        EncoderLayer layer;
        layer.linear.in_features = w_shape[1];
        layer.linear.out_features = w_shape[0];
        layer.activation = nonlinearity;
        
        // Reshape weight matrix: [out_features, in_features]
        layer.linear.weights.resize(w_shape[0]);
        for (int i = 0; i < w_shape[0]; i++) {
            layer.linear.weights[i].resize(w_shape[1]);
            for (int j = 0; j < w_shape[1]; j++) {
                layer.linear.weights[i][j] = w_data[i * w_shape[1] + j];
            }
        }
        
        layer.linear.bias = b_data;
        encoder_layers_.push_back(layer);
    }
    
    return true;
}

bool RLPolicyClean::load_gru_from_json() {
    if (!json_parser_) return false;
    
    std::string w_ih_key = "core.core.weight_ih_l0";
    std::string w_hh_key = "core.core.weight_hh_l0";
    std::string b_ih_key = "core.core.bias_ih_l0";
    std::string b_hh_key = "core.core.bias_hh_l0";
    
    std::vector<float> w_ih_data, w_hh_data, b_ih_data, b_hh_data;
    std::vector<int> w_ih_shape, w_hh_shape, b_ih_shape, b_hh_shape;
    
    if (!json_parser_->extract_tensor(w_ih_key, w_ih_data, w_ih_shape) ||
        !json_parser_->extract_tensor(w_hh_key, w_hh_data, w_hh_shape) ||
        !json_parser_->extract_tensor(b_ih_key, b_ih_data, b_ih_shape) ||
        !json_parser_->extract_tensor(b_hh_key, b_hh_data, b_hh_shape)) {
        return false;
    }
    
    gru_weights_.hidden_size = w_hh_shape[1];
    gru_weights_.input_size = w_ih_shape[1];
    
    // Reshape weight_ih: [3*hidden, input]
    int rows_ih = w_ih_shape[0];
    int cols_ih = w_ih_shape[1];
    gru_weights_.weight_ih.resize(rows_ih);
    for (int i = 0; i < rows_ih; i++) {
        gru_weights_.weight_ih[i].resize(cols_ih);
        for (int j = 0; j < cols_ih; j++) {
            gru_weights_.weight_ih[i][j] = w_ih_data[i * cols_ih + j];
        }
    }
    
    // Reshape weight_hh: [3*hidden, hidden]
    int rows_hh = w_hh_shape[0];
    int cols_hh = w_hh_shape[1];
    gru_weights_.weight_hh.resize(rows_hh);
    for (int i = 0; i < rows_hh; i++) {
        gru_weights_.weight_hh[i].resize(cols_hh);
        for (int j = 0; j < cols_hh; j++) {
            gru_weights_.weight_hh[i][j] = w_hh_data[i * cols_hh + j];
        }
    }
    
    gru_weights_.bias_ih = b_ih_data;
    gru_weights_.bias_hh = b_hh_data;
    gru_hidden_size = gru_weights_.hidden_size;
    
    // Resize hxs to match the loaded hidden size
    hxs = VectorXd::Zero(gru_hidden_size);
    
    // Determine input size from encoder
    if (!encoder_layers_.empty()) {
        gru_input_size = encoder_layers_.back().linear.out_features;
    } else {
        gru_input_size = gru_weights_.input_size;
    }
    
    return true;
}

bool RLPolicyClean::load_dist_linear_from_json() {
    if (!json_parser_) return false;
    
    std::string w_key = "action_parameterization.distribution_linear.weight";
    std::string b_key = "action_parameterization.distribution_linear.bias";
    
    std::vector<float> w_data, b_data;
    std::vector<int> w_shape, b_shape;
    
    if (!json_parser_->extract_tensor(w_key, w_data, w_shape) ||
        !json_parser_->extract_tensor(b_key, b_data, b_shape)) {
        return false;
    }
    
    dist_linear_.in_features = w_shape[1];
    dist_linear_.out_features = w_shape[0];
    
    // Reshape weight matrix: [out_features, in_features]
    dist_linear_.weights.resize(w_shape[0]);
    for (int i = 0; i < w_shape[0]; i++) {
        dist_linear_.weights[i].resize(w_shape[1]);
        for (int j = 0; j < w_shape[1]; j++) {
            dist_linear_.weights[i][j] = w_data[i * w_shape[1] + j];
        }
    }
    
    dist_linear_.bias = b_data;
    return true;
}

double RLPolicyClean::activation_func(double x, Activation act) const {
    switch (act) {
        case Activation::RELU:
            return std::max(0.0, x);
        case Activation::ELU:
            return x > 0.0 ? x : (std::exp(x) - 1.0);
        case Activation::TANH:
            return std::tanh(x);
    }
    return x;
}

VectorXd RLPolicyClean::linear_forward(const LinearLayer& layer, const VectorXd& input) const {
    VectorXd output(layer.out_features);
    
    for (int i = 0; i < layer.out_features; i++) {
        output[i] = layer.bias[i];
        for (int j = 0; j < layer.in_features; j++) {
            output[i] += layer.weights[i][j] * input[j];
        }
    }
    
    return output;
}

VectorXd RLPolicyClean::encoder_forward(const VectorXd& input) const {
    VectorXd x = input;
    
    for (const auto& layer : encoder_layers_) {
        x = linear_forward(layer.linear, x);
        for (int i = 0; i < x.size(); i++) {
            x[i] = activation_func(x[i], layer.activation);
        }
    }
    
    return x;
}

VectorXd RLPolicyClean::normalize_obs(const VectorXd& obs) const {
    VectorXd normalized(obs.size());
    for (int i = 0; i < obs.size(); i++) {
        double std_val = std::sqrt(obs_var[i] + EPS);
        normalized[i] = (obs[i] - obs_mean[i]) / std_val;
        normalized[i] = std::max(-5.0, std::min(5.0, normalized[i]));
    }
    return normalized;
}

VectorXd RLPolicyClean::gru_forward(const VectorXd& input) {
    int hidden = gru_hidden_size;
    
    // Split weights into z, r, n gates (each is hidden_size rows)
    auto get_gate_weights = [](const std::vector<std::vector<float>>& weights, int gate, int hidden_size) {
        std::vector<std::vector<float>> gate_weights(hidden_size);
        for (int i = 0; i < hidden_size; i++) {
            gate_weights[i] = weights[gate * hidden_size + i];
        }
        return gate_weights;
    };
    
    auto get_gate_bias = [](const std::vector<float>& bias, int gate, int hidden_size) {
        std::vector<float> gate_bias(hidden_size);
        for (int i = 0; i < hidden_size; i++) {
            gate_bias[i] = bias[gate * hidden_size + i];
        }
        return gate_bias;
    };
    
    // Matrix-vector multiply
    auto matvec = [](const std::vector<std::vector<float>>& W, const VectorXd& x) {
        VectorXd result(W.size());
        for (size_t i = 0; i < W.size(); i++) {
            result[i] = 0.0;
            for (int j = 0; j < x.size(); j++) {
                result[i] += W[i][j] * x[j];
            }
        }
        return result;
    };
    
    // Element-wise operations
    auto sigmoid = [](double x) { return 1.0 / (1.0 + std::exp(-x)); };
    auto tanh_func = [](double x) { return std::tanh(x); };
    
    // Get gate weights and biases
    // PyTorch GRU gate order is [reset, update, new]
    auto W_r = get_gate_weights(gru_weights_.weight_ih, 0, hidden);  // RESET gate
    auto W_z = get_gate_weights(gru_weights_.weight_ih, 1, hidden);  // UPDATE gate
    auto W_n = get_gate_weights(gru_weights_.weight_ih, 2, hidden);  // NEW gate
    
    auto U_r = get_gate_weights(gru_weights_.weight_hh, 0, hidden);  // RESET gate
    auto U_z = get_gate_weights(gru_weights_.weight_hh, 1, hidden);  // UPDATE gate
    auto U_n = get_gate_weights(gru_weights_.weight_hh, 2, hidden);  // NEW gate
    
    auto b_r_ih = get_gate_bias(gru_weights_.bias_ih, 0, hidden);  // RESET gate
    auto b_z_ih = get_gate_bias(gru_weights_.bias_ih, 1, hidden);  // UPDATE gate
    auto b_n_ih = get_gate_bias(gru_weights_.bias_ih, 2, hidden);  // NEW gate
    
    auto b_r_hh = get_gate_bias(gru_weights_.bias_hh, 0, hidden);  // RESET gate
    auto b_z_hh = get_gate_bias(gru_weights_.bias_hh, 1, hidden);  // UPDATE gate
    auto b_n_hh = get_gate_bias(gru_weights_.bias_hh, 2, hidden);  // NEW gate
    
    // Current hidden state
    VectorXd h_prev = hxs;
    
    // Compute gates
    VectorXd z_t = matvec(W_z, input);
    VectorXd r_t = matvec(W_r, input);
    VectorXd n_t = matvec(W_n, input);
    
    VectorXd z_h = matvec(U_z, h_prev);
    VectorXd r_h = matvec(U_r, h_prev);
    VectorXd n_h = matvec(U_n, h_prev);
    
    // Add biases and apply activations
    VectorXd z(hidden), r(hidden), n(hidden);
    for (int i = 0; i < hidden; i++) {
        z[i] = sigmoid(z_t[i] + z_h[i] + b_z_ih[i] + b_z_hh[i]);
        r[i] = sigmoid(r_t[i] + r_h[i] + b_r_ih[i] + b_r_hh[i]);
        // New gate: tanh(W_n @ x + b_n_ih + r * (U_n @ h + b_n_hh))
        n[i] = tanh_func(n_t[i] + b_n_ih[i] + r[i] * (n_h[i] + b_n_hh[i]));
    }
    
    // Update hidden state (PyTorch GRU formula)
    VectorXd h_next(hidden);
    for (int i = 0; i < hidden; i++) {
        h_next[i] = n[i] + z[i] * (h_prev[i] - n[i]);
    }
    
    hxs = h_next;
    return h_next;
}

std::pair<VectorXd, VectorXd> RLPolicyClean::forward(
    const VectorXd& obs,
    bool normalized) {
    
    // Save the input hidden state BEFORE it gets updated by gru_forward
    // This is the state that was used as input to produce the action
    VectorXd hxs_input = hxs;
    
    // Normalize observation
    VectorXd x = normalized ? obs : normalize_obs(obs);
    
    // Encoder forward
    x = encoder_forward(x);
    
    // GRU forward (this updates hxs to the output state)
    x = gru_forward(x);
    
    // Distribution linear
    VectorXd params = linear_forward(dist_linear_, x);
    
    // Split into mean and logstd, then concatenate
    int action_dim = params.size() / 2;
    VectorXd mean = params.head(action_dim);
    VectorXd logstd = params.tail(action_dim);
    
    VectorXd action_logits(params.size());
    action_logits.head(action_dim) = mean;
    action_logits.tail(action_dim) = logstd;
    
    // Return (action_logits, hxs_output) where hxs_output is the new state
    // But we also need to provide access to hxs_input for logging
    // For now, return the output state (hxs) as before
    return std::make_pair(action_logits, hxs);
}

void RLPolicyClean::reset_hidden_state(int batch_size) {
    (void)batch_size;  // Suppress unused parameter warning
    hxs = VectorXd::Zero(gru_hidden_size);
}

void RLPolicyClean::set_hidden_state(const VectorXd& hxs_vec) {
    if (hxs_vec.size() == gru_hidden_size) {
        hxs = hxs_vec;
    } else {
        std::cerr << "Warning: Hidden state size mismatch" << std::endl;
    }
}

std::shared_ptr<RLPolicyClean> RLPolicyClean::load_from_json(
    const std::string& json_path,
    const std::string& nonlinearity) {
    
    auto policy = std::make_shared<RLPolicyClean>();
    
    // Create and use SimpleJSONParser
    policy->json_parser_ = std::make_shared<SimpleJSONParser>();
    if (!policy->json_parser_->parse_file(json_path)) {
        throw std::runtime_error("Failed to parse JSON file: " + json_path);
    }
    
    // Convert nonlinearity string to enum
    Activation act = Activation::RELU;
    if (nonlinearity == "elu") {
        act = Activation::ELU;
    } else if (nonlinearity == "tanh") {
        act = Activation::TANH;
    }
    
    if (!policy->load_normalizer_from_json()) {
        std::cerr << "Warning: Could not load normalizer, using defaults" << std::endl;
    }
    
    if (!policy->build_encoder_from_json(act)) {
        throw std::runtime_error("Error: Could not build encoder");
    }
    
    if (!policy->load_gru_from_json()) {
        throw std::runtime_error("Error: Could not load GRU");
    }
    
    if (!policy->load_dist_linear_from_json()) {
        throw std::runtime_error("Error: Could not load distribution linear");
    }
    
    policy->reset_hidden_state(1);
    return policy;
}
