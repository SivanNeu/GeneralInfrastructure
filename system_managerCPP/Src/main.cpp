#include "SystemManager.h"
#include "general.h"
#include "utils/TimeUtils.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <regex>
#include <cstdlib>
#include <algorithm>

// Simple JSON parser for config file
struct ConfigParams {
    std::string config_dir;
    std::string log_dir;
    double currentTime = -1.0;
    double dronemass = 0.55;
    Vector3d heading_dir_ned = Vector3d(-1, 1, 0);
    Vector3d desiredHeadingDir_ned = Vector3d(-1, 1, 0);
    MISSION_TYPE missionType = MISSION_TYPE::WAYPOINT;
    YAW_COMMAND yawControlType = YAW_COMMAND::DEFINED_DIR;
    double yawCommandFactor = 1.0;
    double maximalVelocity = 0.75;
    double descentVelocity = 10.0;
    double targetVelocity = 7.5;
    Vector3d originOffset_frd = Vector3d::Zero();
    bool terminalHomingAlowed = true;
    double circleRadius = 5.0;
    CONTROLLER_TYPE controllerType = CONTROLLER_TYPE::VELOCITYRL;
    YAW_COMMAND_TYPE yawCommandType = YAW_COMMAND_TYPE::RATE;
    bool rateControlEnabled = false;
    bool valid = false;
};

// Helper function to parse Vector3d from JSON array [x, y, z]
Vector3d parseVector3d(const std::string& content, const std::string& key) {
    std::regex vec_regex("\"" + key + "\"\\s*:\\s*\\[\\s*(-?\\d+\\.?\\d*)\\s*,\\s*(-?\\d+\\.?\\d*)\\s*,\\s*(-?\\d+\\.?\\d*)\\s*\\]");
    std::smatch match;
    if (std::regex_search(content, match, vec_regex)) {
        try {
            double x = std::stod(match[1].str());
            double y = std::stod(match[2].str());
            double z = std::stod(match[3].str());
            return Vector3d(x, y, z);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse Vector3d for '" << key << "', using default" << std::endl;
        }
    }
    return Vector3d::Zero();
}

// Helper function to parse enum from string
MISSION_TYPE parseMissionType(const std::string& str) {
    if (str == "NONE") return MISSION_TYPE::NONE;
    if (str == "WAYPOINT") return MISSION_TYPE::WAYPOINT;
    if (str == "VELOCITY") return MISSION_TYPE::VELOCITY;
    if (str == "CIRCLE") return MISSION_TYPE::CIRCLE;
    if (str == "LISSAJOUS") return MISSION_TYPE::LISSAJOUS;
    if (str == "TRACKER") return MISSION_TYPE::TRACKER;
    if (str == "SECTION") return MISSION_TYPE::SECTION;
    if (str == "SPINNING") return MISSION_TYPE::SPINNING;
    return MISSION_TYPE::WAYPOINT;
}

YAW_COMMAND parseYawCommand(const std::string& str) {
    if (str == "NO_CONTROL") return YAW_COMMAND::NO_CONTROL;
    if (str == "DEFINED_DIR") return YAW_COMMAND::DEFINED_DIR;
    if (str == "HOLD_CUR_DIR") return YAW_COMMAND::HOLD_CUR_DIR;
    if (str == "VELOCITY_DIR") return YAW_COMMAND::VELOCITY_DIR;
    if (str == "CAMERA_DIR") return YAW_COMMAND::CAMERA_DIR;
    return YAW_COMMAND::DEFINED_DIR;
}

CONTROLLER_TYPE parseControllerType(const std::string& str) {
    if (str == "VELOCITYPID") return CONTROLLER_TYPE::VELOCITYPID;
    if (str == "VELOCITYRL") return CONTROLLER_TYPE::VELOCITYRL;
    if (str == "ACCELERATIONPID") return CONTROLLER_TYPE::ACCELERATIONPID;
    if (str == "GEOMETRIC") return CONTROLLER_TYPE::GEOMETRIC;
    if (str == "ADAPTIVEGEOMETRIC") return CONTROLLER_TYPE::ADAPTIVEGEOMETRIC;
    if (str == "JAEYOUNG") return CONTROLLER_TYPE::JAEYOUNG;
    if (str == "BRESCIANINI") return CONTROLLER_TYPE::BRESCIANINI;
    if (str == "KOOIJMAN") return CONTROLLER_TYPE::KOOIJMAN;
    return CONTROLLER_TYPE::VELOCITYRL;
}

YAW_COMMAND_TYPE parseYawCommandType(const std::string& str) {
    if (str == "NONE") return YAW_COMMAND_TYPE::NONE;
    if (str == "ANGLE") return YAW_COMMAND_TYPE::ANGLE;
    if (str == "RATE") return YAW_COMMAND_TYPE::RATE;
    return YAW_COMMAND_TYPE::RATE;
}

ConfigParams parseConfigFile(const std::string& config_path) {
    ConfigParams params;
    std::ifstream file(config_path);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file: " << config_path << std::endl;
        return params;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();
    
    // Extract config_dir - handle whitespace and quotes
    std::regex config_dir_regex(R"regex("config_dir"\s*:\s*"([^"]+)")regex");
    std::smatch match;
    if (std::regex_search(content, match, config_dir_regex)) {
        params.config_dir = match[1].str();
    } else {
        std::cerr << "Warning: Could not find 'config_dir' in config file" << std::endl;
        return params;
    }
    
    // Extract log_dir
    std::regex log_dir_regex(R"regex("log_dir"\s*:\s*"([^"]+)")regex");
    if (std::regex_search(content, match, log_dir_regex)) {
        params.log_dir = match[1].str();
    } else {
        std::cerr << "Warning: Could not find 'log_dir' in config file" << std::endl;
        return params;
    }
    
    // Extract currentTime (optional, defaults to -1.0)
    std::regex currentTime_regex(R"regex("currentTime"\s*:\s*(-?\d+\.?\d*(?:[eE][+-]?\d+)?))regex");
    if (std::regex_search(content, match, currentTime_regex)) {
        try {
            params.currentTime = std::stod(match[1].str());
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse 'currentTime', using default -1.0" << std::endl;
            params.currentTime = -1.0;
        }
    }
    
    // Extract dronemass
    std::regex dronemass_regex(R"regex("dronemass"\s*:\s*(-?\d+\.?\d*(?:[eE][+-]?\d+)?))regex");
    if (std::regex_search(content, match, dronemass_regex)) {
        try {
            params.dronemass = std::stod(match[1].str());
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse 'dronemass', using default 0.55" << std::endl;
        }
    }
    
    // Extract Vector3d fields
    params.heading_dir_ned = parseVector3d(content, "heading_dir_ned");
    params.desiredHeadingDir_ned = parseVector3d(content, "desiredHeadingDir_ned");
    params.originOffset_frd = parseVector3d(content, "originOffset_frd");
    
    // Extract missionType enum
    std::regex missionType_regex(R"regex("missionType"\s*:\s*"([^"]+)")regex");
    if (std::regex_search(content, match, missionType_regex)) {
        params.missionType = parseMissionType(match[1].str());
    }
    
    // Extract yawControlType enum
    std::regex yawControlType_regex(R"regex("yawControlType"\s*:\s*"([^"]+)")regex");
    if (std::regex_search(content, match, yawControlType_regex)) {
        params.yawControlType = parseYawCommand(match[1].str());
    }
    
    // Extract controllerType enum
    std::regex controllerType_regex(R"regex("controllerType"\s*:\s*"([^"]+)")regex");
    if (std::regex_search(content, match, controllerType_regex)) {
        params.controllerType = parseControllerType(match[1].str());
    }
    
    // Extract yawCommandType enum
    std::regex yawCommandType_regex(R"regex("yawCommandType"\s*:\s*"([^"]+)")regex");
    if (std::regex_search(content, match, yawCommandType_regex)) {
        params.yawCommandType = parseYawCommandType(match[1].str());
    }
    
    // Extract double values
    auto parseDouble = [&content](const std::string& key, double& value, double default_val) {
        std::regex regex("\"" + key + "\"\\s*:\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)");
        std::smatch match;
        if (std::regex_search(content, match, regex)) {
            try {
                value = std::stod(match[1].str());
            } catch (const std::exception& e) {
                value = default_val;
            }
        }
    };
    
    parseDouble("yawCommandFactor", params.yawCommandFactor, 1.0);
    parseDouble("maximalVelocity", params.maximalVelocity, 0.75);
    parseDouble("descentVelocity", params.descentVelocity, 10.0);
    parseDouble("targetVelocity", params.targetVelocity, 7.5);
    parseDouble("circleRadius", params.circleRadius, 5.0);
    
    // Extract boolean values
    auto parseBool = [&content](const std::string& key, bool& value, bool default_val) {
        std::regex regex("\"" + key + "\"\\s*:\\s*(true|false)");
        std::smatch match;
        if (std::regex_search(content, match, regex)) {
            value = (match[1].str() == "true");
        } else {
            value = default_val;
        }
    };
    
    parseBool("terminalHomingAlowed", params.terminalHomingAlowed, true);
    parseBool("rateControlEnabled", params.rateControlEnabled, false);
    
    params.valid = true;
    return params;
}

void printHelp(const char* program_name) {
    std::cout << "System Manager - Drone Control System" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " [OPTIONS] [config_file.json]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help     Show this help message and exit" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  config_file.json    Path to configuration file (JSON format)" << std::endl;
    std::cout << "                      If not specified, searches for 'system_manager_config.json'" << std::endl;
    std::cout << "                      in current directory or parent directory" << std::endl;
    std::cout << std::endl;
    std::cout << "Configuration File:" << std::endl;
    std::cout << "  The configuration file should be a JSON file containing the following fields:" << std::endl;
    std::cout << "    - config_dir (string, required): Directory for configuration files" << std::endl;
    std::cout << "    - log_dir (string, required): Directory for log files" << std::endl;
    std::cout << "    - currentTime (double, optional): Current time, defaults to -1.0" << std::endl;
    std::cout << "    - dronemass (double, optional): Drone mass in kg, defaults to 0.55" << std::endl;
    std::cout << "    - heading_dir_ned (array[3], optional): Heading direction in NED frame" << std::endl;
    std::cout << "    - desiredHeadingDir_ned (array[3], optional): Desired heading direction" << std::endl;
    std::cout << "    - missionType (string, optional): Mission type (NONE, WAYPOINT, VELOCITY, etc.)" << std::endl;
    std::cout << "    - yawControlType (string, optional): Yaw control type" << std::endl;
    std::cout << "    - controllerType (string, optional): Controller type" << std::endl;
    std::cout << "    - yawCommandType (string, optional): Yaw command type (NONE, ANGLE, RATE)" << std::endl;
    std::cout << "    - And other optional parameters..." << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --help" << std::endl;
    std::cout << "  " << program_name << " config.json" << std::endl;
    std::cout << "  " << program_name << "  # Uses default config file search" << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        // Check for help flag
        if (argc > 1) {
            std::string arg = argv[1];
            if (arg == "--help" || arg == "-h") {
                printHelp(argc > 0 ? argv[0] : "SystemManagerMain");
                return 0;
            }
        }
        
        // Parse command-line arguments
        std::string config_file = "system_manager_config.json";
        
        if (argc > 1) {
            config_file = argv[1];
        } else {
            // Try to find config file in current directory or parent directory
            std::ifstream test_file(config_file);
            if (!test_file.is_open()) {
                // Try in parent directory
                config_file = "../system_manager_config.json";
                test_file.open(config_file);
                if (!test_file.is_open()) {
                    config_file = "system_manager_config.json";
                }
            }
            test_file.close();
        }
        
        std::cout << "Loading configuration from: " << config_file << std::endl;
        
        // Parse config file
        ConfigParams config = parseConfigFile(config_file);
        
        if (!config.valid) {
            std::cerr << "Error: Failed to parse config file. Using default values." << std::endl;
            std::cerr << "Usage: " << (argc > 0 ? argv[0] : "SystemManagerMain") 
                      << " [config_file.json]" << std::endl;
            std::cerr << "Default: config_dir='../logs/', log_dir='config/', currentTime=-1.0" << std::endl;
            
            // Use defaults if parsing fails
            config.config_dir = "../logs/";
            config.log_dir = "config/";
            config.currentTime = -1.0;
        } else {
            std::cout << "Config loaded successfully:" << std::endl;
            std::cout << "  config_dir: " << config.config_dir << std::endl;
            std::cout << "  log_dir: " << config.log_dir << std::endl;
            std::cout << "  currentTime: " << config.currentTime << std::endl;
            std::cout << "  dronemass: " << config.dronemass << std::endl;
            std::cout << "  heading_dir_ned: [" << config.heading_dir_ned.transpose() << "]" << std::endl;
            std::cout << "  desiredHeadingDir_ned: [" << config.desiredHeadingDir_ned.transpose() << "]" << std::endl;
            std::cout << "  missionType: " << static_cast<int>(config.missionType) << std::endl;
            std::cout << "  yawControlType: " << static_cast<int>(config.yawControlType) << std::endl;
            std::cout << "  yawCommandFactor: " << config.yawCommandFactor << std::endl;
            std::cout << "  maximalVelocity: " << config.maximalVelocity << std::endl;
            std::cout << "  descentVelocity: " << config.descentVelocity << std::endl;
            std::cout << "  targetVelocity: " << config.targetVelocity << std::endl;
            std::cout << "  originOffset_frd: [" << config.originOffset_frd.transpose() << "]" << std::endl;
            std::cout << "  terminalHomingAlowed: " << (config.terminalHomingAlowed ? "true" : "false") << std::endl;
            std::cout << "  circleRadius: " << config.circleRadius << std::endl;
            std::cout << "  controllerType: " << static_cast<int>(config.controllerType) << std::endl;
            std::cout << "  yawCommandType: " << static_cast<int>(config.yawCommandType) << std::endl;
            std::cout << "  rateControlEnabled: " << (config.rateControlEnabled ? "true" : "false") << std::endl;
        }
        
        // Initialize SystemManager with parameters from config file
        SystemManager sysMgr(config.config_dir, config.log_dir, config.currentTime,
                            config.dronemass,
                            config.heading_dir_ned,
                            config.desiredHeadingDir_ned,
                            config.missionType,
                            config.yawControlType,
                            config.yawCommandFactor,
                            config.maximalVelocity,
                            config.descentVelocity,
                            config.targetVelocity,
                            config.originOffset_frd,
                            config.terminalHomingAlowed,
                            config.circleRadius,
                            config.controllerType,
                            config.yawCommandType,
                            config.rateControlEnabled);
        
        std::cout << "System_Manager initialized. Starting main loop..." << std::endl;
        std::cout << "System_Manager: Waiting for flight data from hardware_adapter..." << std::endl;
        std::cout << "System_Manager: If you see '-return-0-' messages, hardware_adapter may not be publishing data" << std::endl;
        std::cout << "System_Manager: Main loop running at 100Hz..." << std::endl;
        
        int loop_count = 0;
        double next_loop_time = TimeUtils::now();
        const double loop_period = 0.010;  // 100Hz = 0.010 seconds per iteration
        
        while (true) {
            // Variable delay to maintain 100Hz loop frequency
            double current_time = TimeUtils::now();
            double sleep_duration = next_loop_time - current_time;
            
            if (sleep_duration > 0) {
                TimeUtils::sleep(sleep_duration);
            }
            
            next_loop_time += loop_period;
            
            double startTime = TimeUtils::now();
            
            try {
                sysMgr.sys_manager_step();
                loop_count++;
                
                // Print status every 1000 loops (every ~10 seconds at 100Hz)
                if (loop_count % 1000 == 0) {
                    std::cout << "System_Manager: Main loop running, iteration " << loop_count << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error in sys_manager_step: " << e.what() << std::endl;
                // Print stack trace equivalent (C++ doesn't have traceback, but we can print exception info)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Prevent tight error loop
            } catch (...) {
                std::cerr << "Unknown error in sys_manager_step" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Prevent tight error loop
            }
            
            double endTime = TimeUtils::now();
            // Uncomment to print computation time:
            // std::cout << "Computation Time: " << (endTime - startTime) << " seconds" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error initializing SystemManager: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown fatal error initializing SystemManager" << std::endl;
        return 1;
    }
    
    return 0;
}

