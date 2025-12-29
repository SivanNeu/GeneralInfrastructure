#include "RLPolicyJSON.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <fstream>
#include <map>
#include <algorithm>
#include <cctype>

// Helper function to check if file exists
bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

// Parse boolean string (true/false, 1/0, yes/no)
bool parse_bool(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return (lower == "true" || lower == "1" || lower == "yes");
}

// Parse command-line arguments in format --key=value
std::map<std::string, std::string> parse_args(int argc, char* argv[]) {
    std::map<std::string, std::string> args;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.substr(0, 2) == "--") {
            size_t eq_pos = arg.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = arg.substr(2, eq_pos - 2);
                std::string value = arg.substr(eq_pos + 1);
                args[key] = value;
            }
        }
    }
    
    return args;
}

int main(int argc, char* argv[]) {
    // Parse command-line arguments
    auto args = parse_args(argc, argv);
    
    // Check for required arguments
    if (args.find("json") == args.end() || args.find("pth") == args.end()) {
        std::cerr << "Usage: " << argv[0] << " --json=<jsonpath> --pth=<pthpath> [--nonlinearity=<relu|elu|tanh>] [--normalized=<true|false>]" << std::endl;
        std::cerr << "\nRequired arguments:" << std::endl;
        std::cerr << "  --json=<jsonpath>     Path to JSON file (converted from .pth)" << std::endl;
        std::cerr << "  --pth=<pthpath>       Path to PyTorch checkpoint (.pth)" << std::endl;
        std::cerr << "\nOptional arguments:" << std::endl;
        std::cerr << "  --nonlinearity=<type> Activation type: relu|elu|tanh (default: relu)" << std::endl;
        std::cerr << "  --normalized=<bool>   Whether input is normalized: true|false (default: false)" << std::endl;
        return 1;
    }
    
    std::string json_path = args["json"];
    std::string pth_path = args["pth"];
    std::string nonlinearity_str = (args.find("nonlinearity") != args.end()) ? args["nonlinearity"] : "relu";
    bool normalized = (args.find("normalized") != args.end()) ? parse_bool(args["normalized"]) : false;
    
    // Validate nonlinearity
    if (nonlinearity_str != "relu" && nonlinearity_str != "elu" && nonlinearity_str != "tanh") {
        std::cerr << "Error: Invalid nonlinearity: " << nonlinearity_str << std::endl;
        std::cerr << "  Must be one of: relu, elu, tanh" << std::endl;
        return 1;
    }
    
    // Validate that both files exist
    if (!file_exists(pth_path)) {
        std::cerr << "Error: PTH file not found: " << pth_path << std::endl;
        return 1;
    }
    
    if (!file_exists(json_path)) {
        std::cerr << "Error: JSON file not found: " << json_path << std::endl;
        return 1;
    }
    
    std::cout << "Using files:" << std::endl;
    std::cout << "  PTH file: " << pth_path << std::endl;
    std::cout << "  JSON file: " << json_path << std::endl;
    
    RLPolicyJSON::Activation activation = RLPolicyJSON::Activation::RELU;
    if (nonlinearity_str == "elu") {
        activation = RLPolicyJSON::Activation::ELU;
    } else if (nonlinearity_str == "tanh") {
        activation = RLPolicyJSON::Activation::TANH;
    }
    
    // Load policy from JSON
    RLPolicyJSON policy;
    if (!policy.load_from_json(json_path, activation)) {
        std::cerr << "Error: Failed to load policy from " << json_path << std::endl;
        return 1;
    }
    
    std::cout << "\nSuccessfully loaded policy from JSON file: " << json_path << std::endl;
    std::cout << "Observation size: " << policy.get_obs_size() << std::endl;
    std::cout << "Action size: " << policy.get_action_size() << std::endl;
    
    // Reset hidden state
    policy.reset_hidden_state(1);
    
    // Create a dummy observation (zeros)
    std::vector<float> obs(policy.get_obs_size(), 0.0f);
    
    // Forward pass
    std::vector<float> action_logits = policy.forward(obs, normalized);
    
    // Print results
    std::cout << "\nAction logits (mean + logstd):" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < action_logits.size(); i++) {
        std::cout << "  [" << i << "] = " << action_logits[i];
        if (i == action_logits.size() / 2 - 1) {
            std::cout << "  (end of mean)";
        }
        std::cout << std::endl;
    }
    
    // Print hidden state
    std::vector<float> hxs = policy.get_hidden_state();
    std::cout << "\nHidden state (first 10 elements):" << std::endl;
    for (size_t i = 0; i < std::min(hxs.size(), size_t(10)); i++) {
        std::cout << "  hxs[" << i << "] = " << hxs[i] << std::endl;
    }
    if (hxs.size() > 10) {
        std::cout << "  ... (total " << hxs.size() << " elements)" << std::endl;
    }
    
    return 0;
}
