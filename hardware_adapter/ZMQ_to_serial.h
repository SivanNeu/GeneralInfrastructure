#ifndef ZMQ_TO_SERIAL_H
#define ZMQ_TO_SERIAL_H

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

// ZMQ to Serial bridge structure
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
    pthread_t mavlink_sender_thread;    // Process 4: Dequeues from command_queue and sends to MAVLink Serial
    
    // Thread-safe queues - ONLY shared structures between threads
    command_queue_t command_queue;       // ONLY shared structure between process 3 (ZMQ reader) and process 4 (MAVLink sender)
    
    // ZMQ sockets
    void* sub_socket;
    
    // Mavlink address (serial device path)
    char* mavlink_address;
} zmq_to_serial_t;

// Function declarations
int zmq_to_serial_init(zmq_to_serial_t* bridge, const char* log_dir);
void zmq_to_serial_cleanup(zmq_to_serial_t* bridge);
bool zmq_to_serial_init_succeeded(zmq_to_serial_t* bridge);
void zmq_to_serial_stop(zmq_to_serial_t* bridge);

// Internal functions
int zmq_to_serial_init_mavlink(zmq_to_serial_t* bridge);

// Thread functions
void* zmq_to_serial_zmq_reader_thread_func(void* arg);  // Process 3: Read ZMQ, enqueue to command_queue
void* zmq_to_serial_mavlink_sender_thread_func(void* arg);  // Process 4: Dequeue from command_queue, send to MAVLink Serial

#endif // ZMQ_TO_SERIAL_H
