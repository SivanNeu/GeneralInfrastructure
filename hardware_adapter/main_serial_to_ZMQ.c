#define _POSIX_C_SOURCE 200809L
#include "serial_to_ZMQ.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static serial_to_zmq_t* g_bridge = NULL;

void print_help(const char* program_name) {
    printf("Serial to ZMQ Bridge - MAVLink Serial to ZMQ Communication Bridge\n");
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
    printf("  This program acts as a bridge between MAVLink flight controller\n");
    printf("  and the system manager via ZMQ communication. It receives\n");
    printf("  messages from the MAVLink flight controller via Serial port\n");
    printf("  and publishes them to ZMQ for consumption by the system manager.\n");
    printf("\n");
    printf("  Default MAVLink connection: /dev/ttyS0:921600\n");
    printf("\n");
    printf("  The bridge runs a single data thread that:\n");
    printf("    - Receives MAVLink messages from the flight controller via Serial\n");
    printf("    - Parses and filters the data\n");
    printf("    - Publishes flight data to system_manager via ZMQ\n");
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
        printf("\nShutting down Serial to ZMQ bridge...\n");
        serial_to_zmq_stop(g_bridge);
        serial_to_zmq_cleanup(g_bridge);
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
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strncmp(argv[i], "--address=", 10) == 0) {
            // Parse --address=value format
            mavlink_address = argv[i] + 10;  // Skip "--address="
            if (mavlink_address[0] == '\0') {
                fprintf(stderr, "Error: --address= requires a value\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
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
    
    // Allocate Serial to ZMQ bridge (use calloc to ensure zero-initialization)
    g_bridge = (serial_to_zmq_t*)calloc(1, sizeof(serial_to_zmq_t));
    if (g_bridge == NULL) {
        fprintf(stderr, "Failed to allocate Serial to ZMQ bridge\n");
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
    
    // Initialize Serial to ZMQ bridge
    if (serial_to_zmq_init(g_bridge, log_dir) != 0) {
        fprintf(stderr, "Serial to ZMQ bridge initialization failed. Exiting.\n");
        free(g_bridge);
        return 1;
    }
    
    if (!serial_to_zmq_init_succeeded(g_bridge)) {
        fprintf(stderr, "Serial to ZMQ bridge initialization failed. Exiting.\n");
        serial_to_zmq_cleanup(g_bridge);
        free(g_bridge);
        return 1;
    }
    
    printf("Serial to ZMQ bridge started with data thread:\n");
    printf("  - Data thread: receiving MAVLink messages and publishing to system_manager\n");
    
    // Main loop just keeps the process alive
    // The thread handles all the work asynchronously
    while (1) {
        sleep(1);
    }
    
    return 0;
}
