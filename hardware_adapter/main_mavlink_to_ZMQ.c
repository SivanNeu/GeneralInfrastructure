#define _POSIX_C_SOURCE 200809L
#include "mavlink_to_ZMQ.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static mavlink_to_zmq_t* g_bridge = NULL;

void print_help(const char* program_name) {
    printf("MAVLink to ZMQ Bridge - MAVLink UDP to ZMQ Communication Bridge\n");
    printf("\n");
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --help, -h          Show this help message and exit\n");
    printf("  --udp:ADDRESS       MAVLink UDP connection address\n");
    printf("                       Format: --udp:PORT (server mode) or --udp:IP:PORT (client mode)\n");
    printf("                       (default: udp:14540)\n");
    printf("  --zmq:PORT          ZMQ publisher port number\n");
    printf("                       (default: 7790)\n");
    printf("  --log:DIRECTORY     Directory for log files\n");
    printf("                       (default: ../logs/)\n");
    printf("\n");
    printf("Description:\n");
    printf("  This program acts as a bridge between MAVLink flight controller\n");
    printf("  and the system manager via ZMQ communication. It receives\n");
    printf("  messages from the MAVLink flight controller via UDP and publishes\n");
    printf("  them to ZMQ for consumption by the system manager.\n");
    printf("\n");
    printf("  Default MAVLink connection: udp:14540 (server mode, listening on port 14540)\n");
    printf("\n");
    printf("  The bridge runs a single data thread that:\n");
    printf("    - Receives MAVLink messages from the flight controller\n");
    printf("    - Parses and filters the data\n");
    printf("    - Publishes flight data to system_manager via ZMQ\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                                    # Use defaults (../logs/, udp:14540, zmq:7790)\n", program_name);
    printf("  %s --log:./logs                       # Use custom log directory\n", program_name);
    printf("  %s --udp:14541                        # Use different UDP port\n", program_name);
    printf("  %s --zmq:7800                         # Use different ZMQ port\n", program_name);
    printf("  %s --udp:192.168.1.100:14540          # Connect to specific IP:PORT (client mode)\n", program_name);
    printf("  %s --udp:14540 --zmq:7800             # Specify both MAVLink address and ZMQ port\n", program_name);
    printf("  %s --udp:14541 --log:./logs           # Specify address and log directory\n", program_name);
    printf("  %s --zmq:7800 --udp:14541 --log:./logs # Specify all options\n", program_name);
    printf("  %s --help                             # Show this help message\n", program_name);
    printf("\n");
}

void signal_handler(int sig) {
    (void)sig;
    if (g_bridge != NULL) {
        printf("\nShutting down MAVLink to ZMQ bridge...\n");
        mavlink_to_zmq_stop(g_bridge);
        mavlink_to_zmq_cleanup(g_bridge);
        free(g_bridge);
        g_bridge = NULL;
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    const char* mavlink_address = NULL;
    int zmq_port = 0;  // 0 means use default
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strncmp(argv[i], "--zmq:", 6) == 0) {
            // Parse --zmq:portnumber
            const char* port_str = argv[i] + 6;  // Skip "--zmq:"
            if (*port_str == '\0') {
                fprintf(stderr, "Error: --zmq: requires a port number\n");
                fprintf(stderr, "Example: --zmq:7790\n");
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
        } else if (strncmp(argv[i], "--udp:", 6) == 0) {
            // Parse --udp:address
            const char* addr_str = argv[i] + 6;  // Skip "--udp:"
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
        } else if (strncmp(argv[i], "--log:", 6) == 0) {
            // Parse --log:directory
            const char* dir_str = argv[i] + 6;  // Skip "--log:"
            if (*dir_str == '\0') {
                fprintf(stderr, "Error: --log: requires a directory path\n");
                fprintf(stderr, "Example: --log:./logs\n");
                fprintf(stderr, "Use --help for usage information\n");
                return 1;
            }
            log_dir = dir_str;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        } else {
            fprintf(stderr, "Error: Unexpected argument: %s\n", argv[i]);
            fprintf(stderr, "All options must use -- prefix (e.g., --udp:, --zmq:, --log:)\n");
            fprintf(stderr, "Use --help for usage information\n");
            return 1;
        }
    }
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Allocate MAVLink to ZMQ bridge (use calloc to ensure zero-initialization)
    g_bridge = (mavlink_to_zmq_t*)calloc(1, sizeof(mavlink_to_zmq_t));
    if (g_bridge == NULL) {
        fprintf(stderr, "Failed to allocate MAVLink to ZMQ bridge\n");
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
    
    // Set ZMQ port if provided
    if (zmq_port > 0) {
        g_bridge->zmq_port = zmq_port;
    }
    
    // Initialize MAVLink to ZMQ bridge
    if (mavlink_to_zmq_init(g_bridge, log_dir) != 0) {
        fprintf(stderr, "MAVLink to ZMQ bridge initialization failed. Exiting.\n");
        free(g_bridge);
        return 1;
    }
    
    if (!mavlink_to_zmq_init_succeeded(g_bridge)) {
        fprintf(stderr, "MAVLink to ZMQ bridge initialization failed. Exiting.\n");
        mavlink_to_zmq_cleanup(g_bridge);
        free(g_bridge);
        return 1;
    }
    
    printf("MAVLink to ZMQ bridge started with data thread:\n");
    printf("  - Data thread: receiving MAVLink messages and publishing to system_manager\n");
    
    // Main loop just keeps the process alive
    // The thread handles all the work asynchronously
    while (1) {
        sleep(1);
    }
    
    return 0;
}
