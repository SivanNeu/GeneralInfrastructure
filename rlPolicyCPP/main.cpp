#include "RLPolicyJSON.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>
#include <cctype>
#include <cmath>

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

// Generate output filename based on input test data filename
std::string generate_output_filename(const std::string& test_data_path) {
    size_t last_slash = test_data_path.find_last_of("/");
    size_t last_dot = test_data_path.find_last_of(".");
    
    std::string dir = (last_slash != std::string::npos) ? test_data_path.substr(0, last_slash + 1) : "";
    std::string base = (last_slash != std::string::npos) ? test_data_path.substr(last_slash + 1) : test_data_path;
    
    if (last_dot != std::string::npos && last_dot > last_slash) {
        base = test_data_path.substr(last_slash + 1, last_dot - last_slash - 1);
    }
    
    return dir + base + "_results.txt";
}

// Write results to file
bool write_results(const std::string& output_path,
                  const std::vector<std::vector<float>>& observations,
                  const std::vector<std::vector<float>>& ref_actions,
                  const std::vector<std::vector<float>>& calc_actions,
                  bool has_ref_actions) {
    std::ofstream file(output_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open output file: " << output_path << std::endl;
        return false;
    }
    
    if (observations.empty() || calc_actions.empty()) {
        std::cerr << "Error: No data to write" << std::endl;
        return false;
    }
    
    int num_samples = observations.size();
    int obs_dim = observations[0].size();
    int action_dim = calc_actions[0].size();
    int num_action_channels = action_dim / 2;  // action_dim = mean + logstd for each channel
    
    // Write header with dimensions
    file << "# " << num_samples << " " << obs_dim << " " << action_dim;
    if (has_ref_actions && !ref_actions.empty()) {
        file << " " << ref_actions[0].size();
    }
    file << "\n";
    
    // Write column names
    std::vector<std::string> col_names;
    
    // Observation columns
    for (int i = 0; i < obs_dim; i++) {
        col_names.push_back("obs_" + std::to_string(i));
    }
    
    // Reference action columns (if available)
    if (has_ref_actions && !ref_actions.empty()) {
        for (int i = 0; i < num_action_channels; i++) {
            col_names.push_back("mean_" + std::to_string(i) + "_ref");
        }
        for (int i = 0; i < num_action_channels; i++) {
            col_names.push_back("logstd_" + std::to_string(i) + "_ref");
        }
    }
    
    // Calculated action columns
    for (int i = 0; i < num_action_channels; i++) {
        col_names.push_back("mean_" + std::to_string(i) + "_calc");
    }
    for (int i = 0; i < num_action_channels; i++) {
        col_names.push_back("logstd_" + std::to_string(i) + "_calc");
    }
    
    // Difference columns (if reference actions available)
    if (has_ref_actions && !ref_actions.empty()) {
        for (int i = 0; i < num_action_channels; i++) {
            col_names.push_back("mean_" + std::to_string(i) + "_diff");
        }
        for (int i = 0; i < num_action_channels; i++) {
            col_names.push_back("logstd_" + std::to_string(i) + "_diff");
        }
    }
    
    file << "# ";
    for (size_t i = 0; i < col_names.size(); i++) {
        file << col_names[i];
        if (i < col_names.size() - 1) file << " ";
    }
    file << "\n";
    
    // Write data
    file << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < observations.size(); i++) {
        bool first = true;
        
        // Write observation
        for (size_t j = 0; j < observations[i].size(); j++) {
            if (!first) file << " ";
            file << observations[i][j];
            first = false;
        }
        
        // Write reference actions (if available)
        if (has_ref_actions && i < ref_actions.size() && !ref_actions[i].empty()) {
            for (size_t j = 0; j < ref_actions[i].size(); j++) {
                file << " " << ref_actions[i][j];
            }
        }
        
        // Write calculated actions
        if (i < calc_actions.size()) {
            for (size_t j = 0; j < calc_actions[i].size(); j++) {
                file << " " << calc_actions[i][j];
            }
        }
        
        // Write differences (if reference actions available)
        if (has_ref_actions && i < ref_actions.size() && !ref_actions[i].empty() && 
            i < calc_actions.size() && ref_actions[i].size() == calc_actions[i].size()) {
            for (size_t j = 0; j < ref_actions[i].size(); j++) {
                float diff = calc_actions[i][j] - ref_actions[i][j];
                file << " " << diff;
            }
        }
        
        file << "\n";
    }
    
    file.close();
    return true;
}

// Load test data from file
// Format: # num_samples obs_dim [action_dim]
//         # obs_0 obs_1 ... mean_0 logstd_0 ...
//         obs1_1 obs1_2 ... [act1_1 ...]
//         obs2_1 obs2_2 ... [act2_1 ...]
bool load_test_data(const std::string& file_path, 
                   std::vector<std::vector<float>>& observations,
                   std::vector<std::vector<float>>& actions) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open test data file: " << file_path << std::endl;
        return false;
    }
    
    std::string line;
    bool header_found = false;
    int num_samples = 0;
    int obs_dim = 0;
    int action_dim = 0;
    bool has_actions = false;
    
    while (std::getline(file, line)) {
        // Skip empty lines
        if (line.empty()) continue;
        
        // Parse header
        if (line[0] == '#') {
            // Check if this is the dimension header (contains numbers) or column names
            std::istringstream iss(line.substr(1));
            std::string first_token;
            iss >> first_token;
            
            // If first token is a number, it's the dimension header
            if (!first_token.empty() && std::isdigit(first_token[0])) {
                std::istringstream iss2(line.substr(1));
                iss2 >> num_samples >> obs_dim;
                if (iss2 >> action_dim) {
                    has_actions = true;
                }
                header_found = true;
            }
            // Otherwise it's column names, skip it
            continue;
        }
        
        // Parse data lines
        if (header_found) {
            std::istringstream iss(line);
            std::vector<float> obs(obs_dim);
            std::vector<float> act;
            
            // Read observation
            for (int i = 0; i < obs_dim; i++) {
                if (!(iss >> obs[i])) {
                    std::cerr << "Error: Invalid observation data in line" << std::endl;
                    return false;
                }
            }
            observations.push_back(obs);
            
            // Read action if present
            if (has_actions) {
                act.resize(action_dim);
                for (int i = 0; i < action_dim; i++) {
                    if (!(iss >> act[i])) {
                        std::cerr << "Error: Invalid action data in line" << std::endl;
                        return false;
                    }
                }
                actions.push_back(act);
            }
        }
    }
    
    file.close();
    
    if (!header_found) {
        std::cerr << "Error: No header found in test data file" << std::endl;
        return false;
    }
    
    if (static_cast<int>(observations.size()) != num_samples) {
        std::cerr << "Warning: Expected " << num_samples << " samples, got " << observations.size() << std::endl;
    }
    
    return true;
}

int main(int argc, char* argv[]) {
    // Parse command-line arguments
    auto args = parse_args(argc, argv);
    
    // Check for required arguments
    if (args.find("json") == args.end()) {
        std::cerr << "Usage: " << argv[0] << " --json=<jsonpath> [--test-data=<test_data.txt>] [--nonlinearity=<relu|elu|tanh>] [--normalized=<true|false>]" << std::endl;
        std::cerr << "\nRequired arguments:" << std::endl;
        std::cerr << "  --json=<jsonpath>     Path to JSON file (converted from .pth)" << std::endl;
        std::cerr << "\nOptional arguments:" << std::endl;
        std::cerr << "  --test-data=<path>   Path to test data file (generated by generate_test_data.py)" << std::endl;
        std::cerr << "  --nonlinearity=<type> Activation type: relu|elu|tanh (default: relu)" << std::endl;
        std::cerr << "  --normalized=<bool>   Whether input is normalized: true|false (default: false)" << std::endl;
        return 1;
    }
    
    std::string json_path = args["json"];
    std::string test_data_path = (args.find("test-data") != args.end()) ? args["test-data"] : "";
    std::string nonlinearity_str = (args.find("nonlinearity") != args.end()) ? args["nonlinearity"] : "relu";
    bool normalized = (args.find("normalized") != args.end()) ? parse_bool(args["normalized"]) : false;
    
    // Validate nonlinearity
    if (nonlinearity_str != "relu" && nonlinearity_str != "elu" && nonlinearity_str != "tanh") {
        std::cerr << "Error: Invalid nonlinearity: " << nonlinearity_str << std::endl;
        std::cerr << "  Must be one of: relu, elu, tanh" << std::endl;
        return 1;
    }
    
    // Validate that JSON file exists
    if (!file_exists(json_path)) {
        std::cerr << "Error: JSON file not found: " << json_path << std::endl;
        return 1;
    }
    
    std::cout << "Using JSON file: " << json_path << std::endl;
    
    RLPolicyJSON::Activation activation = RLPolicyJSON::Activation::RELU;
    if (nonlinearity_str == "elu") {
        activation = RLPolicyJSON::Activation::ELU;
    } else if (nonlinearity_str == "tanh") {
        activation = RLPolicyJSON::Activation::TANH;
    }
    
    // Load policy from JSON
    std::cout << "\nLoading JSON-based policy..." << std::endl;
    RLPolicyJSON policy_json;
    if (!policy_json.load_from_json(json_path, activation)) {
        std::cerr << "Error: Failed to load policy from " << json_path << std::endl;
        return 1;
    }
    std::cout << "  ✓ JSON policy loaded successfully" << std::endl;
    
    std::cout << "\nPolicy configurations:" << std::endl;
    std::cout << "  Observation size: " << policy_json.get_obs_size() << std::endl;
    std::cout << "  Action size: " << policy_json.get_action_size() << std::endl;
    std::cout << "  Nonlinearity: " << nonlinearity_str << std::endl;
    std::cout << "  Normalized input: " << (normalized ? "yes" : "no") << std::endl;
    
    // Load test data if provided
    std::vector<std::vector<float>> test_observations;
    std::vector<std::vector<float>> test_actions;
    bool use_test_data = false;
    
    if (!test_data_path.empty()) {
        if (!file_exists(test_data_path)) {
            std::cerr << "Error: Test data file not found: " << test_data_path << std::endl;
            return 1;
        }
        
        std::cout << "\nLoading test data from: " << test_data_path << std::endl;
        if (load_test_data(test_data_path, test_observations, test_actions)) {
            std::cout << "  ✓ Loaded " << test_observations.size() << " test observations" << std::endl;
            if (!test_actions.empty()) {
                std::cout << "  ✓ Loaded " << test_actions.size() << " reference actions" << std::endl;
            }
            use_test_data = true;
        } else {
            std::cerr << "  ✗ Failed to load test data, using dummy observation" << std::endl;
        }
    }
    
    // Reset hidden state
    policy_json.reset_hidden_state(1);
    
    if (use_test_data) {
        // Process all test observations
        std::cout << "\n=== Processing Test Observations ===" << std::endl;
        std::cout << std::fixed << std::setprecision(8);
        
        // Store calculated actions for output file
        std::vector<std::vector<float>> calculated_actions;
        
        int num_correct = 0;
        float max_diff = 0.0f;
        float total_diff = 0.0f;
        const float TOLERANCE = 1e-4f;
        
        for (size_t i = 0; i < test_observations.size(); i++) {
            const auto& obs = test_observations[i];
            
            // Verify observation size
            if (static_cast<int>(obs.size()) != policy_json.get_obs_size()) {
                std::cerr << "Error: Observation size mismatch at sample " << i << std::endl;
                std::cerr << "  Expected: " << policy_json.get_obs_size() << ", Got: " << obs.size() << std::endl;
                continue;
            }
            
            // Reset hidden state for each observation (or keep it for sequential processing)
            // policy_json.reset_hidden_state(1);  // Uncomment to reset between samples
            
            // Forward pass
            std::vector<float> action_json = policy_json.forward(obs, normalized);
            std::vector<float> hxs_json = policy_json.get_hidden_state();
            
            // Store calculated action for output file
            calculated_actions.push_back(action_json);
            
            // Compare with reference action if available
            if (i < test_actions.size() && !test_actions[i].empty()) {
                const auto& ref_action = test_actions[i];
                if (ref_action.size() == action_json.size()) {
                    float max_sample_diff = 0.0f;
                    for (size_t j = 0; j < action_json.size(); j++) {
                        float diff = std::abs(action_json[j] - ref_action[j]);
                        max_sample_diff = std::max(max_sample_diff, diff);
                        total_diff += diff;
                    }
                    max_diff = std::max(max_diff, max_sample_diff);
                    
                    if (max_sample_diff < TOLERANCE) {
                        num_correct++;
                    }
                    
                    // Print first few samples with details
                    if (i < 5) {
                        std::cout << "\nSample " << i << ":" << std::endl;
                        std::cout << "  Observation: [";
                        for (size_t k = 0; k < std::min(obs.size(), size_t(5)); k++) {
                            std::cout << obs[k];
                            if (k < std::min(obs.size(), size_t(5)) - 1) std::cout << ", ";
                        }
                        if (obs.size() > 5) std::cout << ", ...";
                        std::cout << "]" << std::endl;
                        std::cout << "  Action (first 5): [";
                        for (size_t k = 0; k < std::min(action_json.size(), size_t(5)); k++) {
                            std::cout << action_json[k];
                            if (k < std::min(action_json.size(), size_t(5)) - 1) std::cout << ", ";
                        }
                        if (action_json.size() > 5) std::cout << ", ...";
                        std::cout << "]" << std::endl;
                        std::cout << "  Max difference: " << max_sample_diff;
                        if (max_sample_diff < TOLERANCE) {
                            std::cout << " ✓";
                        } else {
                            std::cout << " ✗";
                        }
                        std::cout << std::endl;
                    }
                }
            } else {
                // No reference action, just print result
                if (i < 5) {
                    std::cout << "\nSample " << i << ":" << std::endl;
                    std::cout << "  Action (first 5): [";
                    for (size_t k = 0; k < std::min(action_json.size(), size_t(5)); k++) {
                        std::cout << action_json[k];
                        if (k < std::min(action_json.size(), size_t(5)) - 1) std::cout << ", ";
                    }
                    if (action_json.size() > 5) std::cout << ", ...";
                    std::cout << "]" << std::endl;
                }
            }
        }
        
        // Summary
        std::cout << "\n=== Validation Summary ===" << std::endl;
        std::cout << "Total samples processed: " << test_observations.size() << std::endl;
        if (!test_actions.empty()) {
            std::cout << "Samples within tolerance (" << TOLERANCE << "): " << num_correct << " / " << test_observations.size() << std::endl;
            std::cout << "Max absolute difference: " << std::scientific << max_diff << std::endl;
            if (test_observations.size() > 0) {
                std::cout << "Average absolute difference: " << std::scientific << (total_diff / (test_observations.size() * (test_actions.empty() ? 1 : test_actions[0].size()))) << std::endl;
            }
            float accuracy = (num_correct * 100.0f) / test_observations.size();
            std::cout << "Accuracy: " << std::fixed << std::setprecision(2) << accuracy << "%" << std::endl;
        } else {
            std::cout << "No reference actions provided for comparison" << std::endl;
        }
        
        // Write results to file
        std::string output_file = generate_output_filename(test_data_path);
        std::cout << "\nWriting results to: " << output_file << std::endl;
        if (write_results(output_file, test_observations, test_actions, calculated_actions, !test_actions.empty())) {
            std::cout << "  ✓ Results saved successfully" << std::endl;
        } else {
            std::cerr << "  ✗ Failed to save results" << std::endl;
        }
    } else {
        // Use dummy observation
        std::vector<float> obs(policy_json.get_obs_size(), 0.0f);
        
        std::cout << "\n=== Running Forward Pass (Dummy Observation) ===" << std::endl;
        std::vector<float> action_json = policy_json.forward(obs, normalized);
        std::vector<float> hxs_json = policy_json.get_hidden_state();
        
        std::cout << "\n=== Results ===" << std::endl;
        std::cout << "Action logits (mean + logstd, first 10):" << std::endl;
        std::cout << std::fixed << std::setprecision(8);
        for (size_t i = 0; i < std::min(action_json.size(), size_t(10)); i++) {
            std::cout << "  [" << std::setw(3) << i << "] = " << action_json[i];
            if (i == action_json.size() / 2 - 1) {
                std::cout << "  (end of mean)";
            }
            std::cout << std::endl;
        }
        if (action_json.size() > 10) {
            std::cout << "  ... (total " << action_json.size() << " elements)" << std::endl;
        }
        std::cout << "\nHidden state size: " << hxs_json.size() << std::endl;
        std::cout << "Hidden state (first 10 elements):" << std::endl;
        for (size_t i = 0; i < std::min(hxs_json.size(), size_t(10)); i++) {
            std::cout << "  [" << std::setw(3) << i << "] = " << hxs_json[i] << std::endl;
        }
        if (hxs_json.size() > 10) {
            std::cout << "  ... (total " << hxs_json.size() << " elements)" << std::endl;
        }
    }
    
    return 0;
}
