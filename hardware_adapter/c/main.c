#include "hardware_adapter.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

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

int main(int argc, char* argv[]) {
    const char* log_dir = "../logs/";
    
    if (argc > 1) {
        log_dir = argv[1];
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

