#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "sine_test_module.h"
#include "zmq_commands_mavlink.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// MAVLink includes
#include <mavlink/common/mavlink.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// MAVLink connection structure (from zmq_commands_mavlink.c)
typedef struct {
    int fd;
    struct sockaddr_in remote_addr;
    struct sockaddr_in local_addr;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t system_id;
    uint8_t component_id;
    mavlink_status_t status;
    bool connected;
    bool socket_connected;
} mavlink_connection_t;

// Direct MAVLink send function (bypasses hardware_adapter logging to avoid mutex blocking)
static int sine_test_send_setpoint_direct(void* mavlink_conn, uint32_t time_boot_ms, 
                                          uint8_t coordinate_frame, uint16_t type_mask,
                                          const vec3_t* pos, const vec3_t* vel,
                                          const vec3_t* acc, double yaw, double yaw_rate) {
    if (mavlink_conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)mavlink_conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    // In server mode, don't send until we've received a message (to know where to send)
    // Mutex-free read: copy remote_addr atomically to avoid race with data thread update
    struct sockaddr_in remote_addr_copy;
    uint8_t target_system_copy, target_component_copy;
    
    // Copy connection fields atomically (small structs are typically copied atomically)
    memcpy(&remote_addr_copy, &mconn->remote_addr, sizeof(remote_addr_copy));
    target_system_copy = mconn->target_system;  // uint8_t is naturally atomic
    target_component_copy = mconn->target_component;  // uint8_t is naturally atomic
    
    if (remote_addr_copy.sin_addr.s_addr == INADDR_ANY || remote_addr_copy.sin_port == 0) {
        return -1;  // No remote address set yet
    }
    
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t setpoint;
    
    setpoint.time_boot_ms = time_boot_ms;
    setpoint.target_system = target_system_copy;
    setpoint.target_component = target_component_copy;
    setpoint.coordinate_frame = coordinate_frame;
    setpoint.type_mask = type_mask;
    
    setpoint.x = (pos != NULL) ? (float)pos->data[0] : 0.0f;
    setpoint.y = (pos != NULL) ? (float)pos->data[1] : 0.0f;
    setpoint.z = (pos != NULL) ? (float)pos->data[2] : 0.0f;
    
    setpoint.vx = (vel != NULL) ? (float)vel->data[0] : 0.0f;
    setpoint.vy = (vel != NULL) ? (float)vel->data[1] : 0.0f;
    setpoint.vz = (vel != NULL) ? (float)vel->data[2] : 0.0f;
    
    setpoint.afx = (acc != NULL) ? (float)acc->data[0] : 0.0f;
    setpoint.afy = (acc != NULL) ? (float)acc->data[1] : 0.0f;
    setpoint.afz = (acc != NULL) ? (float)acc->data[2] : 0.0f;
    
    setpoint.yaw = (float)yaw;
    setpoint.yaw_rate = (float)yaw_rate;
    
    mavlink_msg_set_position_target_local_ned_encode(mconn->system_id, mconn->component_id, &msg, &setpoint);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Always use sendto for UDP (direct send, no mutex locks)
    // Use the copied remote_addr to avoid race with data thread
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&remote_addr_copy, sizeof(remote_addr_copy));
    
    return (sent == len) ? 0 : -1;
}

int sine_test_init(sine_test_config_t* config, void* adapter) {
    if (config == NULL || adapter == NULL) {
        return -1;
    }
    
    if (!config->enabled) {
        return 0;  // Not enabled, nothing to initialize
    }
    
    // Store adapter pointer
    config->adapter = adapter;
    
    // Extract mavlink_connection from adapter for direct access
    // Access it via offset: mavlink_connection is the second field (after log_dir which is char*)
    // Using offsetof would be cleaner but requires struct definition, so we use pointer arithmetic
    // hardware_adapter_t structure: char* log_dir (offset 0), void* mavlink_connection (offset sizeof(char*))
    void** adapter_ptrs = (void**)adapter;
    config->mavlink_connection = adapter_ptrs[1];  // mavlink_connection is second pointer field
    
    if (config->mavlink_connection == NULL) {
        fprintf(stderr, "Sine test: MAVLink connection not initialized in adapter\n");
        return -1;
    }
    
    config->running = false;
    config->sent_count = 0;
    
    printf("Sine test module initialized (direct command injection mode)\n");
    printf("  Command frequency: %.1f Hz\n", config->cmd_frequency);
    printf("  Duration: %.1f seconds%s\n", config->duration, (config->duration <= 0) ? " (infinite)" : "");
    printf("  Velocity X: amplitude=%.2f m/s, frequency=%.2f Hz\n", config->vx_amplitude, config->vx_frequency);
    printf("  Velocity Y: amplitude=%.2f m/s, frequency=%.2f Hz\n", config->vy_amplitude, config->vy_frequency);
    printf("  Velocity Z: amplitude=%.2f m/s, frequency=%.2f Hz\n", config->vz_amplitude, config->vz_frequency);
    printf("  Yaw rate: amplitude=%.2f rad/s, frequency=%.2f Hz\n", config->yaw_rate_amplitude, config->yaw_rate_frequency);
    
    return 0;
}

int sine_test_run(sine_test_config_t* config) {
    if (config == NULL || !config->enabled || config->mavlink_connection == NULL) {
        return -1;
    }
    
    config->running = true;
    config->sent_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &config->start_time);
    
    double dt = 1.0 / config->cmd_frequency;
    struct timespec last_print_time = config->start_time;
    
    // Rate-based timing: calculate next command time and sleep precisely until then
    // This prevents timing drift and ensures consistent command intervals
    // Initialize next_cmd_time to start_time + dt so first command is sent immediately
    struct timespec next_cmd_time = config->start_time;
    long dt_nsec = (long)(dt * 1e9);
    next_cmd_time.tv_nsec += dt_nsec;
    if (next_cmd_time.tv_nsec >= 1000000000L) {
        next_cmd_time.tv_sec += next_cmd_time.tv_nsec / 1000000000L;
        next_cmd_time.tv_nsec %= 1000000000L;
    }
    
    printf("Sine test started (direct command injection, rate-based timing)\n");
    
    while (config->running) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        // Wait until it's time for the next command (rate-based timing)
        // This ensures consistent intervals even if command sending takes variable time
        struct timespec sleep_time;
        if (now.tv_sec < next_cmd_time.tv_sec || 
            (now.tv_sec == next_cmd_time.tv_sec && now.tv_nsec < next_cmd_time.tv_nsec)) {
            // Need to wait until next_cmd_time
            sleep_time.tv_sec = next_cmd_time.tv_sec - now.tv_sec;
            sleep_time.tv_nsec = next_cmd_time.tv_nsec - now.tv_nsec;
            if (sleep_time.tv_nsec < 0) {
                sleep_time.tv_sec--;
                sleep_time.tv_nsec += 1000000000L;
            }
            
            // Use clock_nanosleep for precise timing (more accurate than usleep)
            // Only sleep if there's meaningful time (> 100us) to avoid busy-waiting
            if (sleep_time.tv_sec > 0 || sleep_time.tv_nsec > 100000L) {
                clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
                clock_gettime(CLOCK_MONOTONIC, &now);
            }
        }
        
        // Calculate elapsed time from start (after sleep to get accurate time)
        double t = (now.tv_sec - config->start_time.tv_sec) + 
                   (now.tv_nsec - config->start_time.tv_nsec) / 1e9;
        
        // Check if duration has expired
        if (config->duration > 0 && t >= config->duration) {
            printf("Sine test: Duration expired (%.1f seconds), stopping\n", config->duration);
            break;
        }
        
        // Compute sine wave values based on actual elapsed time
        double vx = config->vx_amplitude * sin(2.0 * M_PI * config->vx_frequency * t);
        double vy = config->vy_amplitude * sin(2.0 * M_PI * config->vy_frequency * t);
        double vz = config->vz_amplitude * sin(2.0 * M_PI * config->vz_frequency * t);
        double yaw_rate = config->yaw_rate_amplitude * sin(2.0 * M_PI * config->yaw_rate_frequency * t);
        
        // Create velocity vector
        vec3_t vel;
        vel.data[0] = vx;
        vel.data[1] = vy;
        vel.data[2] = vz;
        
        // Send command directly via MAVLink (bypassing hardware_adapter logging to avoid mutex blocking)
        // This matches the approach used in simpleMavlinkTest for consistent timing
        // Direct send avoids mutex locks in hardware_adapter_log_mavlink_command which can cause timing gaps
        uint32_t time_boot_ms = (uint32_t)(t * 1000.0);
        uint8_t coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
        uint16_t type_mask = 0x07 | 0x1C0 | 0x400;  // ignore pos, acc, yaw; use vel and yaw_rate
        
        // Send directly via MAVLink (no mutex locks, no logging overhead)
        if (sine_test_send_setpoint_direct(config->mavlink_connection, time_boot_ms, coordinate_frame,
                                          type_mask, NULL, &vel, NULL, NAN, yaw_rate) == 0) {
            config->sent_count++;
        }
        
        // Calculate next command time (rate-based, accounts for any delay in sending)
        next_cmd_time.tv_nsec += dt_nsec;
        if (next_cmd_time.tv_nsec >= 1000000000L) {
            next_cmd_time.tv_sec += next_cmd_time.tv_nsec / 1000000000L;
            next_cmd_time.tv_nsec %= 1000000000L;
        }
        
        // Periodically print status (every 5 seconds)
        double elapsed = (now.tv_sec - last_print_time.tv_sec) + 
                        (now.tv_nsec - last_print_time.tv_nsec) / 1e9;
        if (elapsed >= 5.0) {
            printf("Sine test: t=%.2fs, cmd #%d, vel=(%.3f, %.3f, %.3f) m/s, yaw_rate=%.3f rad/s\n",
                   t, config->sent_count, vx, vy, vz, yaw_rate);
            last_print_time = now;
        }
    }
    
    config->running = false;
    printf("Sine test finished, sent %d commands\n", config->sent_count);
    
    return 0;
}

void sine_test_stop(sine_test_config_t* config) {
    if (config != NULL) {
        config->running = false;
    }
}

void sine_test_cleanup(sine_test_config_t* config) {
    if (config == NULL) {
        return;
    }
    
    sine_test_stop(config);
    
    // Clear pointers
    config->adapter = NULL;
    config->mavlink_connection = NULL;
    
    printf("Sine test module cleaned up\n");
}
