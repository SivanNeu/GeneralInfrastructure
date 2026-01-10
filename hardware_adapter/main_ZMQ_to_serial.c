#define _POSIX_C_SOURCE 200809L
#include "ZMQ_to_serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static zmq_to_serial_t* g_bridge = NULL;

void print_help(const char* program_name) {
    printf("ZMQ to Serial Bridge - MAVLink Serial Communication Bridge\n");
    printf("\n");
    printf("Usage: %s [OPTIONS] [LOG_DIRECTORY]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --help, -h          Show this help message and exit\n");
    printf("  --address=ADDRESS   MAVLink Serial connection address\n");
    printf("                      Format: /dev/ttyS0:921600\n");
    printf("                      (default: /dev/ttyS0:921600)\n");
    printf("\n");
    printf("Arguments:\n");
    printf("  LOG_DIRECTORY        Directory for log files (default: ../logs/)\n");
    printf("\n");
    printf("Description:\n");
    printf("  This program acts as a bridge between the system manager and the\n");
    printf("  MAVLink flight controller via Serial communication. It receives\n");
    printf("  commands from the system manager via ZMQ and forwards them to\n");
    printf("  the flight controller via MAVLink over Serial port.\n");
    printf("\n");
    printf("  Default MAVLink connection: /dev/ttyS0:921600\n");
    printf("\n");
    printf("  The bridge runs two threads:\n");
    printf("    - ZMQ reader thread: reads commands from ZMQ and enqueues to command queue\n");
    printf("    - MAVLink sender thread: dequeues commands and sends to MAVLink Serial\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                                    # Use defaults (../logs/, /dev/ttyS0:921600)\n", program_name);
    printf("  %s ./logs                             # Use custom log directory\n", program_name);
    printf("  %s --address=/dev/ttyUSB0:57600       # Use different serial device and baudrate\n", program_name);
    printf("  %s --address=/dev/ttyS0:921600 ./logs # Specify both address and log directory\n", program_name);
    printf("  %s --help                             # Show this help message\n", program_name);
    printf("\n");
}

void signal_handler(int sig) {
    (void)sig;
    if (g_bridge != NULL) {
        printf("\nShutting down ZMQ to Serial bridge...\n");
        zmq_to_serial_stop(g_bridge);
        zmq_to_serial_cleanup(g_bridge);
        free(g_bridge);
        g_bridge = NULL;
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    const char* mavlink_address = NULL;
    
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
        } else if (strncmp(arg, "--address", 9) == 0) {
            if (param_value != NULL) {
                mavlink_address = param_value;
            } else {
                fprintf(stderr, "Error: --address requires a value (use --address=ADDRESS)\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
        } else if (arg[0] == '-') {
            fprintf(stderr, "Unknown option: %s\n", arg);
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        } else {
            // First non-option argument is the log directory
            log_dir = arg;
        }
        
        // Restore '=' if we modified the string (for error messages)
        if (equals != NULL) {
            *equals = '=';
        }
    }
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Allocate ZMQ to Serial bridge (use calloc to ensure zero-initialization)
    g_bridge = (zmq_to_serial_t*)calloc(1, sizeof(zmq_to_serial_t));
    if (g_bridge == NULL) {
        fprintf(stderr, "Failed to allocate ZMQ to Serial bridge\n");
        return 1;
    }
    
    // Set MAVLink address if provided
    if (mavlink_address != NULL) {
        g_bridge->mavlink_address = strdup(mavlink_address);
        if (g_bridge->mavlink_address == NULL) {
            fprintf(stderr, "Failed to allocate memory for MAVLink address\n");
            free(g_bridge);
            return 1;
        }
    }
    
    // Initialize ZMQ to Serial bridge
    if (zmq_to_serial_init(g_bridge, log_dir) != 0) {
        fprintf(stderr, "ZMQ to Serial bridge initialization failed. Exiting.\n");
        free(g_bridge);
        return 1;
    }
    
    if (!zmq_to_serial_init_succeeded(g_bridge)) {
        fprintf(stderr, "ZMQ to Serial bridge initialization failed. Exiting.\n");
        zmq_to_serial_cleanup(g_bridge);
        free(g_bridge);
        return 1;
    }
    
    printf("ZMQ to Serial bridge started with two threads:\n");
    printf("  - ZMQ reader thread: reading commands from system_manager and enqueuing to command queue\n");
    printf("  - MAVLink sender thread: dequeuing commands and sending to MAVLink Serial\n");
    
    // Main loop just keeps the process alive
    // The threads handle all the work asynchronously
    while (1) {
        sleep(1);
    }
    
    return 0;
}
