#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "hardware_adapter.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

static hardware_adapter_t* g_adapter = NULL;

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

static void print_usage(const char* program_name) {
    fprintf(stderr, "Usage: %s [--serial:deviceport:baudrate] [--udp:address:port] [log_dir]\n", program_name);
    fprintf(stderr, "  --serial:deviceport:baudrate  Connect via serial port (e.g., --serial:/dev/ttyS0:921600)\n");
    fprintf(stderr, "  --udp:address:port            Connect via UDP (e.g., --udp:127.0.0.1:14540)\n");
    fprintf(stderr, "  log_dir                       Optional log directory (default: ../logs/)\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s --serial:/dev/ttyS0:921600\n", program_name);
    fprintf(stderr, "  %s --udp:127.0.0.1:14540\n", program_name);
    fprintf(stderr, "  %s --serial:/dev/ttyS0:57600 ../logs/\n", program_name);
}

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    char* mavlink_address = NULL;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--serial:", 9) == 0) {
            // Format: --serial:/dev/ttyS0:921600
            const char* serial_spec = argv[i] + 9;  // Skip "--serial:"
            char* colon = strchr(serial_spec, ':');
            if (colon == NULL) {
                fprintf(stderr, "Error: Invalid serial format. Expected --serial:deviceport:baudrate\n");
                print_usage(argv[0]);
                return 1;
            }
            
            size_t device_len = colon - serial_spec;
            char* device = (char*)malloc(device_len + 1);
            if (device == NULL) {
                fprintf(stderr, "Error: Failed to allocate memory\n");
                return 1;
            }
            strncpy(device, serial_spec, device_len);
            device[device_len] = '\0';
            
            int baudrate = atoi(colon + 1);
            if (baudrate <= 0) {
                fprintf(stderr, "Error: Invalid baudrate: %s\n", colon + 1);
                free(device);
                print_usage(argv[0]);
                return 1;
            }
            
            // Format: serial:///dev/ttyS0:921600
            size_t addr_len = strlen("serial://") + strlen(device) + 1 + 20;  // +1 for ':', +20 for baudrate
            mavlink_address = (char*)malloc(addr_len);
            if (mavlink_address == NULL) {
                fprintf(stderr, "Error: Failed to allocate memory\n");
                free(device);
                return 1;
            }
            snprintf(mavlink_address, addr_len, "serial://%s:%d", device, baudrate);
            free(device);
        }
        else if (strncmp(argv[i], "--udp:", 6) == 0) {
            // Format: --udp:127.0.0.1:14540
            const char* udp_spec = argv[i] + 6;  // Skip "--udp:"
            char* colon = strchr(udp_spec, ':');
            if (colon == NULL) {
                fprintf(stderr, "Error: Invalid UDP format. Expected --udp:address:port\n");
                print_usage(argv[0]);
                return 1;
            }
            
            size_t ip_len = colon - udp_spec;
            char* ip = (char*)malloc(ip_len + 1);
            if (ip == NULL) {
                fprintf(stderr, "Error: Failed to allocate memory\n");
                return 1;
            }
            strncpy(ip, udp_spec, ip_len);
            ip[ip_len] = '\0';
            
            int port = atoi(colon + 1);
            if (port <= 0 || port > 65535) {
                fprintf(stderr, "Error: Invalid port: %s\n", colon + 1);
                free(ip);
                print_usage(argv[0]);
                return 1;
            }
            
            // Format: udp:127.0.0.1:14540
            size_t addr_len = strlen("udp:") + strlen(ip) + 1 + 10;  // +1 for ':', +10 for port
            mavlink_address = (char*)malloc(addr_len);
            if (mavlink_address == NULL) {
                fprintf(stderr, "Error: Failed to allocate memory\n");
                free(ip);
                return 1;
            }
            snprintf(mavlink_address, addr_len, "udp:%s:%d", ip, port);
            free(ip);
        }
        else if (strncmp(argv[i], "--", 2) == 0) {
            fprintf(stderr, "Error: Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
        else {
            // Assume it's the log directory
            log_dir = argv[i];
        }
    }
    
    // Default to serial if no connection type specified
    if (mavlink_address == NULL) {
        mavlink_address = strdup("serial:///dev/ttyS0:57600");
        if (mavlink_address == NULL) {
            fprintf(stderr, "Error: Failed to allocate memory\n");
            return 1;
        }
    }
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Allocate hardware adapter (use calloc to ensure zero-initialization)
    g_adapter = (hardware_adapter_t*)calloc(1, sizeof(hardware_adapter_t));
    if (g_adapter == NULL) {
        fprintf(stderr, "Failed to allocate hardware adapter\n");
        free(mavlink_address);
        return 1;
    }
    
    // Set mavlink address before initialization
    g_adapter->mavlink_address = mavlink_address;
    
    // Initialize hardware adapter
    if (hardware_adapter_init(g_adapter, log_dir) != 0) {
        fprintf(stderr, "Hardware adapter initialization failed. Exiting.\n");
        free(mavlink_address);
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
    printf("  - MAVLink connection: %s\n", mavlink_address);
    
    // Main loop just keeps the process alive
    // The threads handle all the work asynchronously
    while (1) {
        sleep(1);
    }
    
    return 0;
}

