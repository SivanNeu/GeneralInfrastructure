#define _POSIX_C_SOURCE 200809L
#include "hardware_adapterUDP.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static hardware_adapter_t* g_adapter = NULL;

void print_help(const char* program_name) {
    printf("Hardware Adapter (UDP) - MAVLink UDP Communication Bridge\n");
    printf("\n");
    printf("Usage: %s [OPTIONS] [LOG_DIRECTORY]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --help, -h          Show this help message and exit\n");
    printf("  --address ADDRESS   MAVLink UDP connection address\n");
    printf("                      Format: udp:PORT (server mode) or udp:IP:PORT (client mode)\n");
    printf("                      (default: udp:14540)\n");
    printf("\n");
    printf("Arguments:\n");
    printf("  LOG_DIRECTORY        Directory for log files (default: ../logs/)\n");
    printf("\n");
    printf("Description:\n");
    printf("  This program acts as a bridge between the system manager and the\n");
    printf("  MAVLink flight controller via UDP communication. It receives\n");
    printf("  commands from the system manager via ZMQ and forwards them to\n");
    printf("  the flight controller via MAVLink over UDP.\n");
    printf("\n");
    printf("  Default MAVLink connection: udp:14540 (server mode, listening on port 14540)\n");
    printf("\n");
    printf("  The adapter runs two threads:\n");
    printf("    - Command thread: handles commands from system_manager to MAVLink\n");
    printf("    - Data thread: maintains current_data and publishes to system_manager\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                                    # Use defaults (../logs/, udp:14540)\n", program_name);
    printf("  %s ./logs                             # Use custom log directory\n", program_name);
    printf("  %s --address udp:14550                # Use different UDP port (server mode)\n", program_name);
    printf("  %s --address udp:192.168.1.100:14540 # Connect to specific IP:PORT (client mode)\n", program_name);
    printf("  %s --address udp:14540 ./logs         # Specify both address and log directory\n", program_name);
    printf("  %s --help                             # Show this help message\n", program_name);
    printf("\n");
}

void signal_handler(int sig) {
    (void)sig;
    if (g_adapter != NULL) {
        printf("\nShutting down hardware adapter...\n");
        hardware_adapter_stop(g_adapter);
        hardware_adapter_cleanup(g_adapter);
        free(g_adapter);
        g_adapter = NULL;
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    const char* mavlink_address = NULL;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--address") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: --address requires an argument\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            mavlink_address = argv[++i];
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        } else {
            // First non-option argument is the log directory
            log_dir = argv[i];
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
    
    printf("Hardware adapter started with two threads:\n");
    printf("  - Command thread: handling commands from system_manager to mavlink\n");
    printf("  - Data thread: maintaining current_data and publishing to system_manager\n");
    
    // Main loop just keeps the process alive
    // The threads handle all the work asynchronously
    while (1) {
        sleep(1);
    }
    
    return 0;
}

