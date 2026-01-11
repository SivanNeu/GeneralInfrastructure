#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include "common.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Constants
#define COMMAND_QUEUE_SIZE 256  // Size of command queue

// Command types
typedef enum {
    CMD_TYPE_VEL_NED,      // Velocity command in NED frame
    CMD_TYPE_VEL_BODY,     // Velocity command in body frame (needs quaternion)
    CMD_TYPE_ATTITUDE,     // Attitude command (quaternion)
    CMD_TYPE_ATTITUDE_RATE, // Attitude rate command
    CMD_TYPE_ARM,          // Arm command
    CMD_TYPE_ACC           // Acceleration command (not implemented)
} command_type_t;

// Command structure - contains all data needed to send a command
typedef struct {
    command_type_t type;
    
    // For velocity commands
    vec3_t command;
    double yaw;
    double yaw_rate;
    quaternion_t quat_ned_bodyfrd;  // For VEL_BODY: quaternion to transform body to NED
    
    // For attitude commands
    quaternion_t quat;
    vec3_t rpy_rate;
    double thrust;
    bool is_rate;  // true for rate command, false for quaternion command
} command_t;

// Thread-safe command queue
typedef struct {
    command_t commands[COMMAND_QUEUE_SIZE];
    volatile size_t head;  // Write index (producer)
    volatile size_t tail;  // Read index (consumer)
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;  // Signal when queue has items
    pthread_cond_t not_full;   // Signal when queue has space
} command_queue_t;

// Queue functions
int command_queue_init(command_queue_t* queue);
void command_queue_cleanup(command_queue_t* queue);
int command_queue_enqueue(command_queue_t* queue, const command_t* cmd);
int command_queue_dequeue(command_queue_t* queue, command_t* cmd, int timeout_ms);
size_t command_queue_size(command_queue_t* queue);

#endif // COMMAND_QUEUE_H
