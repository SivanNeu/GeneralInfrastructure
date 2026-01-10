#include "SystemManager.h"
#include "general.h"
#include "utils/TimeUtils.h"
#include "utils/JsonFile.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <algorithm>

// Simple JSON parser for config file
struct ConfigParams {
    std::string log_dir;
    double currentTime = DEFAULT_CURRENT_TIME;
    double dronemass = DEFAULT_DRONE_MASS;
    Vector3d heading_dir_ned = Vector3d(DEFAULT_HEADING_DIR_NED_X, DEFAULT_HEADING_DIR_NED_Y, DEFAULT_HEADING_DIR_NED_Z);
    Vector3d desiredHeadingDir_ned = Vector3d(DEFAULT_DESIRED_HEADING_DIR_NED_X, DEFAULT_DESIRED_HEADING_DIR_NED_Y, DEFAULT_DESIRED_HEADING_DIR_NED_Z);
    MISSION_TYPE missionType = MISSION_TYPE::WAYPOINT;
    YAW_COMMAND yawControlType = YAW_COMMAND::DEFINED_DIR;
    double yawCommandFactor = DEFAULT_YAW_COMMAND_FACTOR;
    double maximalVelocity = DEFAULT_MAXIMAL_VELOCITY;
    double descentVelocity = DEFAULT_DESCENT_VELOCITY;
    double targetVelocity = DEFAULT_TARGET_VELOCITY;
    Vector3d originOffset_frd = Vector3d::Zero();
    bool terminalHomingAlowed = DEFAULT_TERMINAL_HOMING_ALLOWED;
    double circleRadius = DEFAULT_CIRCLE_RADIUS;
    std::vector<Vector3d> waypointList;  // List of waypoints for WAYPOINT mission type
    double waypointReachThreshold = DEFAULT_WAYPOINT_REACH_THRESHOLD;  // Distance threshold to consider waypoint reached (meters)
    CONTROLLER_TYPE controllerType = CONTROLLER_TYPE::VELOCITYRL;
    CONTROLLER_TYPE primaryControllerType = CONTROLLER_TYPE::VELOCITYRL;
    std::string primaryControllerParamsFile;
    CONTROLLER_TYPE secondaryControllerType = CONTROLLER_TYPE::VELOCITYPID;
    std::string secondaryControllerParamsFile;
    YAW_COMMAND_TYPE yawCommandType = YAW_COMMAND_TYPE::RATE;
    bool rateControlEnabled = DEFAULT_RATE_CONTROL_ENABLED;
    double loop_period = DEFAULT_LOOP_PERIOD;
    bool valid = false;
};

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
    
    // Load JSON file using JsonFile class
    JsonFile json(config_path);
    if (!json.isValid()) {
        std::cerr << "Error: Could not open config file: " << config_path << std::endl;
        std::cerr << "Exiting program." << std::endl;
        std::exit(1);
    }
    
    // Extract required string fields
    params.log_dir = json.getString("log_dir", "");
    if (params.log_dir.empty()) {
        std::cerr << "Error: Could not find 'log_dir' in config file: " << config_path << std::endl;
        std::cerr << "Exiting program." << std::endl;
        std::exit(1);
    }
    
    // Extract double values
    params.currentTime = json.getDouble("currentTime", DEFAULT_CURRENT_TIME);
    params.dronemass = json.getDouble("dronemass", DEFAULT_DRONE_MASS);
    params.yawCommandFactor = json.getDouble("yawCommandFactor", DEFAULT_YAW_COMMAND_FACTOR);
    params.maximalVelocity = json.getDouble("maximalVelocity", DEFAULT_MAXIMAL_VELOCITY);
    params.descentVelocity = json.getDouble("descentVelocity", DEFAULT_DESCENT_VELOCITY);
    params.targetVelocity = json.getDouble("targetVelocity", DEFAULT_TARGET_VELOCITY);
    params.circleRadius = json.getDouble("circleRadius", DEFAULT_CIRCLE_RADIUS);
    
    // Extract Vector3d fields
    params.heading_dir_ned = json.getVector3d("heading_dir_ned", Vector3d(DEFAULT_HEADING_DIR_NED_X, DEFAULT_HEADING_DIR_NED_Y, DEFAULT_HEADING_DIR_NED_Z));
    params.desiredHeadingDir_ned = json.getVector3d("desiredHeadingDir_ned", Vector3d(DEFAULT_DESIRED_HEADING_DIR_NED_X, DEFAULT_DESIRED_HEADING_DIR_NED_Y, DEFAULT_DESIRED_HEADING_DIR_NED_Z));
    params.originOffset_frd = json.getVector3d("originOffset_frd", Vector3d::Zero());
    
    // Extract waypoint list (array of Vector3d)
    params.waypointList = json.getVector3dArray("waypointList");
    params.waypointReachThreshold = json.getDouble("waypointReachThreshold", DEFAULT_WAYPOINT_REACH_THRESHOLD);
    
    // Extract boolean values
    params.terminalHomingAlowed = json.getBool("terminalHomingAlowed", DEFAULT_TERMINAL_HOMING_ALLOWED);
    params.rateControlEnabled = json.getBool("rateControlEnabled", DEFAULT_RATE_CONTROL_ENABLED);
    
    // Extract enum values (need to parse strings)
    std::string missionTypeStr = json.getString("missionType", "WAYPOINT");
    params.missionType = parseMissionType(missionTypeStr);
    
    std::string yawControlTypeStr = json.getString("yawControlType", "DEFINED_DIR");
    params.yawControlType = parseYawCommand(yawControlTypeStr);
    
    std::string controllerTypeStr = json.getString("controllerType", "VELOCITYRL");
    params.controllerType = parseControllerType(controllerTypeStr);
    
    std::string primaryControllerTypeStr = json.getString("primaryControllerType", "");
    if (!primaryControllerTypeStr.empty()) {
        params.primaryControllerType = parseControllerType(primaryControllerTypeStr);
    } else {
        // Fall back to controllerType if primaryControllerType not specified
        params.primaryControllerType = params.controllerType;
    }
    
    params.primaryControllerParamsFile = json.getString("primaryControllerParamsFile", "");
    
    std::string secondaryControllerTypeStr = json.getString("secondaryControllerType", "");
    if (!secondaryControllerTypeStr.empty()) {
        params.secondaryControllerType = parseControllerType(secondaryControllerTypeStr);
    }
    // If empty string, keep default (VELOCITYPID)
    
    params.secondaryControllerParamsFile = json.getString("secondaryControllerParamsFile", "");
    
    std::string yawCommandTypeStr = json.getString("yawCommandType", "RATE");
    params.yawCommandType = parseYawCommandType(yawCommandTypeStr);
    
    params.loop_period = json.getDouble("loop_period", DEFAULT_LOOP_PERIOD);
    
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
            std::cerr << "Default: log_dir='config/', currentTime=" << DEFAULT_CURRENT_TIME << std::endl;
            
            // Use defaults if parsing fails
            config.log_dir = "config/";
            config.currentTime = DEFAULT_CURRENT_TIME;
        } else {
            std::cout << "Config loaded successfully:" << std::endl;
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
            std::cout << "  waypointList size: " << config.waypointList.size() << std::endl;
            if (!config.waypointList.empty()) {
                for (size_t i = 0; i < config.waypointList.size(); i++) {
                    std::cout << "    waypoint[" << i << "]: [" << config.waypointList[i].transpose() << "]" << std::endl;
                }
            }
            std::cout << "  waypointReachThreshold: " << config.waypointReachThreshold << std::endl;
            std::cout << "  controllerType: " << static_cast<int>(config.controllerType) << std::endl;
            std::cout << "  primaryControllerType: " << static_cast<int>(config.primaryControllerType) << std::endl;
            std::cout << "  primaryControllerParamsFile: " << config.primaryControllerParamsFile << std::endl;
            std::cout << "  secondaryControllerType: " << static_cast<int>(config.secondaryControllerType) << std::endl;
            std::cout << "  secondaryControllerParamsFile: " << config.secondaryControllerParamsFile << std::endl;
            std::cout << "  yawCommandType: " << static_cast<int>(config.yawCommandType) << std::endl;
            std::cout << "  rateControlEnabled: " << (config.rateControlEnabled ? "true" : "false") << std::endl;
            std::cout << "  loop_period: " << config.loop_period << std::endl;
        }
        
        // Initialize SystemManager with parameters from config file
        SystemManager sysMgr(config.log_dir, config.currentTime,
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
                            config.waypointList,
                            config.waypointReachThreshold,
                            config.controllerType,
                            config.primaryControllerType,
                            config.primaryControllerParamsFile,
                            config.secondaryControllerType,
                            config.secondaryControllerParamsFile,
                            config.yawCommandType,
                            config.rateControlEnabled);
        
        std::cout << "System_Manager initialized. Starting main loop..." << std::endl;
        std::cout << "System_Manager: Waiting for flight data from hardware_adapter..." << std::endl;
        std::cout << "System_Manager: If you see '-return-0-' messages, hardware_adapter may not be publishing data" << std::endl;
        std::cout << "System_Manager: Main loop running at " << (1.0 / config.loop_period) << "Hz (period: " << config.loop_period << "s)..." << std::endl;
        
        int loop_count = 0;
        double next_loop_time = TimeUtils::now();
        const double loop_period = config.loop_period;
        
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

