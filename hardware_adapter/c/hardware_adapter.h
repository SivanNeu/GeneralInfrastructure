#ifndef HARDWARE_ADAPTER_H
#define HARDWARE_ADAPTER_H

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
#define TIME_BETWEEN_MODE_SET_ATTEMPTS 1
#define ARM_LOOP_DELAY 0.02
#define MAX_VERTICAL_VEL_JUMP_M_S 3.0
#define PUBLISH_FREQ_HZ 500
#define PUBLISH_DT (1.0 / PUBLISH_FREQ_HZ)

// Hardware adapter structure
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
    pthread_t command_thread;
    pthread_t data_thread;
    
    // ZMQ sockets
    void* pub_socket;
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
void hardware_adapter_listener_to_mavlink(hardware_adapter_t* adapter, bool blocking, double timeout, bool use_lock, bool apply_filter);
void hardware_adapter_listener_to_commands(hardware_adapter_t* adapter);
void hardware_adapter_filter_data(hardware_adapter_t* adapter, flight_data_t* current_data);
void hardware_adapter_parse(hardware_adapter_t* adapter, const char* msg_type, void* msg_dict);
void hardware_adapter_send_setpoint(hardware_adapter_t* adapter, const vec3_t* pos, const vec3_t* vel, 
                                     const vec3_t* acc, double yaw, double yaw_rate);
void hardware_adapter_send_goal_attitude(hardware_adapter_t* adapter, double goal_thrust, 
                                          const quaternion_t* goal_attitude, const rate_cmd_t* rates);
void hardware_adapter_send_offboard_cmd(hardware_adapter_t* adapter);
void hardware_adapter_arm(hardware_adapter_t* adapter);
void hardware_adapter_send_takeoff_cmd(hardware_adapter_t* adapter, double takeoff_altitude);
void hardware_adapter_send_land_cmd(hardware_adapter_t* adapter);

// Thread functions
void* hardware_adapter_command_thread_func(void* arg);
void* hardware_adapter_data_thread_func(void* arg);

#endif // HARDWARE_ADAPTER_H

