#ifndef HARDWARE_ADAPTER_H
#define HARDWARE_ADAPTER_H

#include "common.h"
#include "zmq_topics.h"
#include "zmq_wrapper.h"
#include "command_queue.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

// Constants
#define MAVLINK_QUEUE_SIZE 200
#define MAVLINK_RATE_HZ 50
#define MUTEX_TIMEOUT_SEC 1
#define TIME_BETWEEN_MODE_SET_ATTEMPTS 1
#define ARM_LOOP_DELAY 0.02
#define MAX_VERTICAL_VEL_JUMP_M_S 3.0
#define PUBLISH_FREQ_HZ 500
#define PUBLISH_DT (1.0 / PUBLISH_FREQ_HZ)
#define MAVLINK_DEFAULT_UDP_PORT 14030

// Process pair enable/disable defines
// Pair 2: Process 3 (ZMQ reader) + Process 4 (MAVLink sender)
// If not defined, defaults to enabled (1)
#ifndef ENABLE_COMMAND_PROCESSING_PAIR
#define ENABLE_COMMAND_PROCESSING_PAIR 1  // Enable Pair 2: Processes 3 & 4 (ZMQ reader + MAVLink sender)
#endif

// Hardware adapter structure
typedef struct {
    char* log_dir;
    
    // Mavlink connection
    void* mavlink_connection;  // Will be mavlink_connection_t from mavlink library
    
    // Control flags
    bool mavlink_connected_to_usb;
    bool disable_offboard_control;
    bool offboard_control_enabled;
    bool init_success;
    bool running;
    
    // Threading (2 processes)
    pthread_t zmq_reader_thread;        // Process 3: Reads commands from ZMQ and enqueues to command_queue
    pthread_t mavlink_sender_thread;    // Process 4: Dequeues from command_queue and sends to MAVLink UDP
    
    // Thread-safe queues - ONLY shared structures between threads
    command_queue_t command_queue;       // ONLY shared structure between process 3 (ZMQ reader) and process 4 (MAVLink sender)
    
    // ZMQ sockets
    void* sub_socket;
    
    // Mavlink address
    char* mavlink_address;
} hardware_adapter_t;

// Function declarations
int hardware_adapter_init(hardware_adapter_t* adapter, const char* log_dir);
void hardware_adapter_cleanup(hardware_adapter_t* adapter);
bool hardware_adapter_init_succeeded(hardware_adapter_t* adapter);
void hardware_adapter_stop(hardware_adapter_t* adapter);

// Internal functions
int hardware_adapter_init_mavlink(hardware_adapter_t* adapter);
void hardware_adapter_listener_to_commands(hardware_adapter_t* adapter);
void hardware_adapter_send_setpoint(hardware_adapter_t* adapter, const vec3_t* pos, const vec3_t* vel, 
                                     const vec3_t* acc, double yaw, double yaw_rate);
void hardware_adapter_send_goal_attitude(hardware_adapter_t* adapter, double goal_thrust, 
                                          const quaternion_t* goal_attitude, const rate_cmd_t* rates);
void hardware_adapter_send_offboard_cmd(hardware_adapter_t* adapter);
void hardware_adapter_arm(hardware_adapter_t* adapter);
void hardware_adapter_send_takeoff_cmd(hardware_adapter_t* adapter, double takeoff_altitude);
void hardware_adapter_send_land_cmd(hardware_adapter_t* adapter);

// Thread functions (2 processes)
void* hardware_adapter_zmq_reader_thread_func(void* arg);  // Process 3: Read ZMQ, enqueue to command_queue
void* hardware_adapter_mavlink_sender_thread_func(void* arg);  // Process 4: Dequeue from command_queue, send to MAVLink UDP

#endif // HARDWARE_ADAPTER_H
