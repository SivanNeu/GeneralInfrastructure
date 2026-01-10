#ifndef SERIAL_TO_ZMQ_H
#define SERIAL_TO_ZMQ_H

#include "common.h"
#include "low_pass_filter.h"
#include "zmq_topics.h"
#include "zmq_wrapper.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

// Constants
#define MAVLINK_QUEUE_SIZE 200
#define MAVLINK_RATE_HZ 50
#define MUTEX_TIMEOUT_SEC 1
#define MAX_VERTICAL_VEL_JUMP_M_S 3.0
#define PUBLISH_FREQ_HZ 500
#define PUBLISH_DT (1.0 / PUBLISH_FREQ_HZ)

// Serial to ZMQ bridge structure
typedef struct {
    char* log_dir;
    
    // Mavlink connection
    void* mavlink_connection;  // Will be mavlink_connection_t from mavlink library
    
    // Flight data
    flight_data_t current_data;
    pthread_mutex_t data_lock;
    
    // Filters
    low_pass_filter_t vertical_speed_filter;
    low_pass_filter_t altitude_filter;
    double prev_alt_m;
    int alt_vel_count;
    double prev_vel_vertical;
    double prev_alt_ts;
    
    // Control flags
    bool mavlink_connected_to_usb;
    bool disable_offboard_control;
    bool offboard_control_enabled;
    bool init_success;
    bool running;
    
    // Threading
    pthread_t data_thread;
    
    // ZMQ sockets
    void* pub_socket;
    
    // Mavlink address (serial device path)
    char* mavlink_address;
} serial_to_zmq_t;

// Function declarations
int serial_to_zmq_init(serial_to_zmq_t* bridge, const char* log_dir);
void serial_to_zmq_cleanup(serial_to_zmq_t* bridge);
bool serial_to_zmq_init_succeeded(serial_to_zmq_t* bridge);
void serial_to_zmq_stop(serial_to_zmq_t* bridge);

// Internal functions
int serial_to_zmq_init_mavlink(serial_to_zmq_t* bridge);
void serial_to_zmq_listener_to_mavlink(serial_to_zmq_t* bridge, bool blocking, double timeout, bool use_lock, bool apply_filter);
void serial_to_zmq_filter_data(serial_to_zmq_t* bridge, flight_data_t* current_data);
void serial_to_zmq_parse(serial_to_zmq_t* bridge, const char* msg_type, void* msg_dict);

// Thread functions
void* serial_to_zmq_data_thread_func(void* arg);

#endif // SERIAL_TO_ZMQ_H
