#ifndef RL_POLICY_CLEAN_H
#define RL_POLICY_CLEAN_H
#include "general.h"

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <cmath>

// Forward declaration
class SimpleJSONParser;

class RLPolicyClean {
public:
    enum class Activation {
        RELU,
        ELU,
        TANH
    };

    // JSON parsing structures (needed for implementation)
    struct JSONValue {
        enum Type { OBJECT, ARRAY, STRING, NUMBER, BOOL, NULL_VAL };
        Type type;
        std::map<std::string, std::shared_ptr<JSONValue>> object;
        std::vector<std::shared_ptr<JSONValue>> array;
        std::string string_val;
        double number_val;
        bool bool_val;
        
        JSONValue() : type(NULL_VAL), number_val(0.0), bool_val(false) {}
    };

private:
    // Observation normalization parameters
    VectorXd obs_mean;
    VectorXd obs_var;
    
    // Neural network structures (pure C++)
    struct LinearLayer {
        std::vector<std::vector<float>> weights;  // [out_features, in_features]
        std::vector<float> bias;                   // [out_features]
        int in_features;
        int out_features;
    };

    struct EncoderLayer {
        LinearLayer linear;
        Activation activation;
    };

    struct GRUWeights {
        std::vector<std::vector<float>> weight_ih;  // [3*hidden, input]
        std::vector<std::vector<float>> weight_hh;  // [3*hidden, hidden]
        std::vector<float> bias_ih;                 // [3*hidden]
        std::vector<float> bias_hh;                 // [3*hidden]
        int input_size;
        int hidden_size;
    };
    
    // Policy components
    std::vector<EncoderLayer> encoder_layers_;
    GRUWeights gru_weights_;
    LinearLayer dist_linear_;
    
    // Internal recurrent state
    VectorXd hxs;
    
    // Dimensions
    int gru_hidden_size;
    int gru_input_size;
    
    // Constants
    static constexpr double EPS = 1e-5;
    
    // JSON parser instance
    std::shared_ptr<class SimpleJSONParser> json_parser_;
    
    // Helper methods for building network from JSON data
    bool build_encoder_from_json(Activation nonlinearity);
    bool load_gru_from_json();
    bool load_dist_linear_from_json();
    bool load_normalizer_from_json();
    
    // Helper methods for inference
    double activation_func(double x, Activation act) const;
    VectorXd linear_forward(const LinearLayer& layer, const VectorXd& input) const;
    VectorXd encoder_forward(const VectorXd& input) const;
    VectorXd gru_forward(const VectorXd& input);
    VectorXd normalize_obs(const VectorXd& obs) const;

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
    
    // Load policy from JSON file (created by pth2json.py)
    static std::shared_ptr<RLPolicyClean> load_from_json(
        const std::string& json_path,
        const std::string& nonlinearity = "relu");
    
    // Get observation size
    int get_obs_size() const { return static_cast<int>(obs_mean.size()); }
    
    // Get hidden state
    VectorXd get_hidden_state() const { return hxs; }
};

#endif // RL_POLICY_CLEAN_H

