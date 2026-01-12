#define _POSIX_C_SOURCE 200809L
#include "zmq_commands_mavlink.h"
#include "sine_test_module.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

static hardware_adapter_t* g_adapter = NULL;
static sine_test_config_t g_sine_test_config = {0};
static pthread_t g_sine_test_thread = 0;

void print_help(const char* program_name) {
    printf("Hardware Adapter (UDP) - MAVLink UDP Communication Bridge\n");
    printf("\n");
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --help, -h              Show this help message and exit\n");
    printf("  --udp:ADDRESS           MAVLink UDP connection address\n");
    printf("                          Format: --udp:PORT (server mode) or --udp:IP:PORT (client mode)\n");
    printf("                          (default: udp:%d)\n", MAVLINK_DEFAULT_UDP_PORT);
    printf("  --zmq:PORT              ZMQ subscriber port number\n");
    printf("                          (default: %d)\n", DEFAULT_ZMQ_PORT);
    printf("  --log:DIRECTORY         Directory for log files\n");
    printf("                          (default: ../logs/)\n");
    printf("\n");
    printf("Sine Test Module Options:\n");
    printf("  --sine-test                    Enable sine wave command test mode\n");
    printf("  --sine-cmd-freq=FREQ           Command generation frequency in Hz (default: %.1f)\n", SINE_TEST_DEFAULT_CMD_FREQUENCY);
    printf("  --sine-duration=SEC            Test duration in seconds, 0 = infinite (default: %.1f)\n", SINE_TEST_DEFAULT_DURATION);
    printf("  --sine-vx-ampl=AMPL            Velocity X amplitude in m/s (default: %.1f)\n", SINE_TEST_DEFAULT_VX_AMPLITUDE);
    printf("  --sine-vx-freq=FREQ            Velocity X frequency in Hz (default: %.1f)\n", SINE_TEST_DEFAULT_VX_FREQUENCY);
    printf("  --sine-vy-ampl=AMPL            Velocity Y amplitude in m/s (default: %.1f)\n", SINE_TEST_DEFAULT_VY_AMPLITUDE);
    printf("  --sine-vy-freq=FREQ            Velocity Y frequency in Hz (default: %.1f)\n", SINE_TEST_DEFAULT_VY_FREQUENCY);
    printf("  --sine-vz-ampl=AMPL            Velocity Z amplitude in m/s (default: %.1f)\n", SINE_TEST_DEFAULT_VZ_AMPLITUDE);
    printf("  --sine-vz-freq=FREQ            Velocity Z frequency in Hz (default: %.1f)\n", SINE_TEST_DEFAULT_VZ_FREQUENCY);
    printf("  --sine-yawrate-ampl=AMPL       Yaw rate amplitude in rad/s (default: %.1f)\n", SINE_TEST_DEFAULT_YAWRATE_AMPLITUDE);
    printf("  --sine-yawrate-freq=FREQ       Yaw rate frequency in Hz (default: %.1f)\n", SINE_TEST_DEFAULT_YAWRATE_FREQUENCY);
    printf("\n");
    printf("\n");
    printf("Description:\n");
    printf("  This program acts as a bridge between the system manager and the\n");
    printf("  MAVLink flight controller via UDP communication. It receives\n");
    printf("  commands from the system manager via ZMQ and forwards them to\n");
    printf("  the flight controller via MAVLink over UDP.\n");
    printf("\n");
    printf("  Default MAVLink connection: udp:%d (server mode, listening on port %d)\n", 
           MAVLINK_DEFAULT_UDP_PORT, MAVLINK_DEFAULT_UDP_PORT);
    printf("\n");
    printf("  The adapter runs two threads:\n");
    printf("    - Command thread: handles commands from system_manager to MAVLink\n");
    printf("    - Data thread: maintains current_data and publishes to system_manager\n");
    printf("\n");
    printf("  When --sine-test is enabled, the adapter generates sine wave commands\n");
    printf("  for velocity X, Y, Z and yaw rate, sending them via ZMQ to the command thread.\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                                    # Use defaults (../logs/, udp:%d, zmq:%d)\n", 
           program_name, MAVLINK_DEFAULT_UDP_PORT, DEFAULT_ZMQ_PORT);
    printf("  %s --log:./logs                       # Use custom log directory\n", program_name);
    printf("  %s --udp:14550                        # Use different UDP port (server mode)\n", program_name);
    printf("  %s --zmq:7800                         # Use different ZMQ port\n", program_name);
    printf("  %s --udp:14550 --zmq:7800             # Specify both MAVLink address and ZMQ port\n", program_name);
    printf("  %s --udp:14550 --zmq:7800 --log:./logs # Specify all options\n", program_name);
    printf("  %s --sine-test --sine-duration=30     # Run sine test for 30 seconds\n", program_name);
    printf("  %s --sine-test --sine-cmd-freq=100     # Run sine test at 100 Hz command rate\n", program_name);
    printf("  %s --help                             # Show this help message\n", program_name);
    printf("\n");
}

void signal_handler(int sig) {
    (void)sig;
    if (g_sine_test_config.enabled) {
        printf("\nStopping sine test...\n");
        sine_test_stop(&g_sine_test_config);
        if (g_sine_test_thread != 0) {
            pthread_join(g_sine_test_thread, NULL);
            g_sine_test_thread = 0;
        }
        sine_test_cleanup(&g_sine_test_config);
    }
    if (g_adapter != NULL) {
        printf("\nShutting down hardware adapter...\n");
        hardware_adapter_stop(g_adapter);
        hardware_adapter_cleanup(g_adapter);
        free(g_adapter);
        g_adapter = NULL;
    }
    exit(0);
}

// Thread function for running sine test
static void* sine_test_thread_func(void* arg) {
    sine_test_config_t* config = (sine_test_config_t*)arg;
    sine_test_run(config);
    return NULL;
}

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    const char* mavlink_address = NULL;
    int zmq_port = 0;  // 0 means use default
    
    // Initialize sine test config with defaults
    memset(&g_sine_test_config, 0, sizeof(g_sine_test_config));
    g_sine_test_config.enabled = false;
    g_sine_test_config.cmd_frequency = SINE_TEST_DEFAULT_CMD_FREQUENCY;
    g_sine_test_config.duration = SINE_TEST_DEFAULT_DURATION;
    g_sine_test_config.vx_amplitude = SINE_TEST_DEFAULT_VX_AMPLITUDE;
    g_sine_test_config.vx_frequency = SINE_TEST_DEFAULT_VX_FREQUENCY;
    g_sine_test_config.vy_amplitude = SINE_TEST_DEFAULT_VY_AMPLITUDE;
    g_sine_test_config.vy_frequency = SINE_TEST_DEFAULT_VY_FREQUENCY;
    g_sine_test_config.vz_amplitude = SINE_TEST_DEFAULT_VZ_AMPLITUDE;
    g_sine_test_config.vz_frequency = SINE_TEST_DEFAULT_VZ_FREQUENCY;
    g_sine_test_config.yaw_rate_amplitude = SINE_TEST_DEFAULT_YAWRATE_AMPLITUDE;
    g_sine_test_config.yaw_rate_frequency = SINE_TEST_DEFAULT_YAWRATE_FREQUENCY;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        char* arg = argv[i];
        char* param_value = NULL;
        
        // Check if argument contains '=' (format: --paramname=value)
        char* equals = strchr(arg, '=');
        if (equals != NULL) {
            // Split into parameter name and value
            *equals = '\0';
            param_value = equals + 1;
        }
        
        // Handle flags (no value) and parameters with =value format
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strncmp(arg, "--udp:", 6) == 0) {
            // Parse --udp:address
            const char* addr_str = arg + 6;  // Skip "--udp:"
            if (*addr_str == '\0') {
                fprintf(stderr, "Error: --udp: requires an address\n");
                fprintf(stderr, "Example: --udp:14540 or --udp:192.168.1.100:14540\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            if (mavlink_address != NULL) {
                fprintf(stderr, "Error: Multiple MAVLink addresses specified\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            // Construct the address with "udp:" prefix (without the "--")
            char addr_buf[256];
            snprintf(addr_buf, sizeof(addr_buf), "udp:%s", addr_str);
            mavlink_address = strdup(addr_buf);
            if (mavlink_address == NULL) {
                fprintf(stderr, "Failed to allocate memory for MAVLink address\n");
                return 1;
            }
        } else if (strncmp(arg, "--zmq:", 6) == 0) {
            // Parse --zmq:portnumber
            const char* port_str = arg + 6;  // Skip "--zmq:"
            if (*port_str == '\0') {
                fprintf(stderr, "Error: --zmq: requires a port number\n");
                fprintf(stderr, "Example: --zmq:7793\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            zmq_port = atoi(port_str);
            if (zmq_port <= 0 || zmq_port > 65535) {
                fprintf(stderr, "Error: Invalid ZMQ port number: %s\n", port_str);
                fprintf(stderr, "Port must be between 1 and 65535\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
        } else if (strncmp(arg, "--log:", 6) == 0) {
            // Parse --log:directory
            const char* dir_str = arg + 6;  // Skip "--log:"
            if (*dir_str == '\0') {
                fprintf(stderr, "Error: --log: requires a directory path\n");
                fprintf(stderr, "Example: --log:./logs\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            log_dir = dir_str;
        } else if (strcmp(arg, "--sine-test") == 0) {
            g_sine_test_config.enabled = true;
        } else if (strncmp(arg, "--sine-cmd-freq", 15) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.cmd_frequency = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-cmd-freq requires a value (use --sine-cmd-freq=FREQ)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-duration", 15) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.duration = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-duration requires a value (use --sine-duration=SEC)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vx-ampl", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vx_amplitude = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vx-ampl requires a value (use --sine-vx-ampl=AMPL)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vx-freq", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vx_frequency = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vx-freq requires a value (use --sine-vx-freq=FREQ)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vy-ampl", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vy_amplitude = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vy-ampl requires a value (use --sine-vy-ampl=AMPL)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vy-freq", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vy_frequency = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vy-freq requires a value (use --sine-vy-freq=FREQ)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vz-ampl", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vz_amplitude = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vz-ampl requires a value (use --sine-vz-ampl=AMPL)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-vz-freq", 14) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.vz_frequency = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-vz-freq requires a value (use --sine-vz-freq=FREQ)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-yawrate-ampl", 19) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.yaw_rate_amplitude = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-yawrate-ampl requires a value (use --sine-yawrate-ampl=AMPL)\n");
                return 1;
            }
        } else if (strncmp(arg, "--sine-yawrate-freq", 19) == 0) {
            if (param_value != NULL) {
                g_sine_test_config.yaw_rate_frequency = atof(param_value);
            } else {
                fprintf(stderr, "Error: --sine-yawrate-freq requires a value (use --sine-yawrate-freq=FREQ)\n");
                return 1;
            }
        } else if (arg[0] == '-') {
            fprintf(stderr, "Unknown option: %s\n", arg);
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        } else {
            fprintf(stderr, "Error: Unexpected argument: %s\n", arg);
            fprintf(stderr, "All options must use -- prefix (e.g., --udp:, --zmq:, --log:)\n");
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        }
        
        // Restore '=' if we modified the string (for error messages)
        if (equals != NULL) {
            *equals = '=';
        }
    }
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Allocate hardware adapter (use calloc to ensure zero-initialization)
    g_adapter = (hardware_adapter_t*)calloc(1, sizeof(hardware_adapter_t));
    if (g_adapter == NULL) {
        fprintf(stderr, "Failed to allocate hardware adapter\n");
        return 1;
    }
    
    // Set MAVLink address if provided
    if (mavlink_address != NULL) {
        g_adapter->mavlink_address = strdup(mavlink_address);
        if (g_adapter->mavlink_address == NULL) {
            fprintf(stderr, "Failed to allocate memory for MAVLink address\n");
            free(g_adapter);
            return 1;
        }
    }
    
    // Set ZMQ port if provided
    if (zmq_port > 0) {
        g_adapter->zmq_port = zmq_port;
    }
    
    // Configure sine test mode if enabled
    // Thread enable/disable is now controlled only by compile-time defines
    if (g_sine_test_config.enabled) {
#if ENABLE_DATA_PROCESSING_PAIR
        g_adapter->sine_test_mode = true;  // Mark sine test mode for data thread optimization
        printf("Sine test mode enabled:\n");
        printf("  - Commands sent directly via MAVLink (bypassing ZMQ)\n");
#if ENABLE_COMMAND_PROCESSING_PAIR
        printf("  - Command processing pair: DISABLED at compile time (ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
#else
        printf("  - Command processing pair: enabled (processes 3 & 4 will run)\n");
#endif
        printf("  - Data processing pair: enabled (processes 1 & 2 will run)\n");
        printf("  - Data thread will minimize CPU usage (skip ZMQ publishing, filtering)\n");
        printf("  - No mutexes used - relies on thread-safe socket operations and careful data access\n");
#else
        printf("Sine test mode enabled:\n");
        printf("  - Commands sent directly via MAVLink (bypassing ZMQ)\n");
#if ENABLE_COMMAND_PROCESSING_PAIR
        printf("  - Command processing pair: enabled (processes 3 & 4 will run)\n");
#else
        printf("  - Command processing pair: DISABLED at compile time (ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
#endif
        printf("  - Data processing pair: DISABLED at compile time (ENABLE_DATA_PROCESSING_PAIR=0)\n");
        printf("  - WARNING: Cannot receive MAVLink data (data processing pair disabled)\n");
#endif
    }
    
    // Initialize hardware adapter
    if (hardware_adapter_init(g_adapter, log_dir) != 0) {
        fprintf(stderr, "Hardware adapter initialization failed. Exiting.\n");
        free(g_adapter);
        return 1;
    }
    
    if (!hardware_adapter_init_succeeded(g_adapter)) {
        fprintf(stderr, "Hardware adapter initialization failed. Exiting.\n");
        hardware_adapter_cleanup(g_adapter);
        free(g_adapter);
        return 1;
    }
    
    // Print status based on compile-time defines only
#if ENABLE_DATA_PROCESSING_PAIR && ENABLE_COMMAND_PROCESSING_PAIR
    printf("Hardware adapter started with all processes enabled:\n");
    printf("  - Command processing pair: enabled (processes 3 & 4)\n");
    printf("  - Data processing pair: enabled (processes 1 & 2)\n");
    printf("  - Command thread: handling commands from system_manager to mavlink\n");
    printf("  - Data thread: maintaining current_data and publishing to system_manager\n");
#elif ENABLE_COMMAND_PROCESSING_PAIR
    printf("Hardware adapter started with command processing pair only:\n");
    printf("  - Command processing pair: enabled (processes 3 & 4)\n");
    printf("  - Data processing pair: DISABLED (compile-time: ENABLE_DATA_PROCESSING_PAIR=0)\n");
    printf("  - Command thread: handling commands from system_manager to mavlink\n");
    printf("  - Data thread: DISABLED (compile-time: ENABLE_DATA_PROCESSING_PAIR=0)\n");
#elif ENABLE_DATA_PROCESSING_PAIR
    printf("Hardware adapter started with data processing pair only:\n");
    printf("  - Command processing pair: DISABLED (compile-time: ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
    printf("  - Data processing pair: enabled (processes 1 & 2)\n");
    printf("  - Command thread: DISABLED (compile-time: ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
    printf("  - Data thread: maintaining current_data and publishing to system_manager\n");
#else
    printf("Hardware adapter started:\n");
    printf("  - Command processing pair: DISABLED (compile-time: ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
    printf("  - Data processing pair: DISABLED (compile-time: ENABLE_DATA_PROCESSING_PAIR=0)\n");
    printf("  - Command thread: DISABLED (compile-time: ENABLE_COMMAND_PROCESSING_PAIR=0)\n");
    printf("  - Data thread: DISABLED (compile-time: ENABLE_DATA_PROCESSING_PAIR=0)\n");
#endif
    
    // Initialize and start sine test if enabled
    if (g_sine_test_config.enabled) {
        if (sine_test_init(&g_sine_test_config, g_adapter) != 0) {
            fprintf(stderr, "Failed to initialize sine test module\n");
            hardware_adapter_cleanup(g_adapter);
            free(g_adapter);
            return 1;
        }
        
        // Start sine test in a separate thread
        if (pthread_create(&g_sine_test_thread, NULL, sine_test_thread_func, &g_sine_test_config) != 0) {
            fprintf(stderr, "Failed to create sine test thread\n");
            sine_test_cleanup(&g_sine_test_config);
            hardware_adapter_cleanup(g_adapter);
            free(g_adapter);
            return 1;
        }
        printf("Sine test thread started\n");
    }
    
    // Main loop just keeps the process alive
    // The threads handle all the work asynchronously
    while (1) {
        sleep(1);
        
        // Check if sine test has finished (if it was running with finite duration)
        if (g_sine_test_config.enabled && !g_sine_test_config.running && g_sine_test_thread != 0) {
            pthread_join(g_sine_test_thread, NULL);
            g_sine_test_thread = 0;
            printf("Sine test completed, continuing normal operation\n");
            // Note: We continue running the hardware adapter even after sine test finishes
        }
    }
    
    return 0;
}

