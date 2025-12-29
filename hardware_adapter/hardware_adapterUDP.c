#define _POSIX_C_SOURCE 200809L
#include "hardware_adapter.h"
#include "zmq_topics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/errno.h>

// MAVLink includes - adjust path based on your MAVLink installation
// If MAVLink is installed system-wide, use:
#include <mavlink/common/mavlink.h>
// If MAVLink is in a local directory, comment the above and use:
// #include "mavlink/v2.0/common/mavlink.h"
// Then compile with: make MAVLINK_INCLUDE=-I/path/to/mavlink/include

// MAVLink constants (if not defined in headers)
#ifndef MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 255
#endif
#ifndef MAV_COMP_ID_ONBOARD_COMPUTER
#define MAV_COMP_ID_ONBOARD_COMPUTER 191
#endif

// MAVLink connection structure
typedef struct {
    int fd;  // UDP socket file descriptor
    struct sockaddr_in remote_addr;
    struct sockaddr_in local_addr;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t system_id;
    uint8_t component_id;
    mavlink_status_t status;
    bool connected;  // MAVLink connection status (heartbeat received)
    bool socket_connected;  // UDP socket connected state
} mavlink_connection_t;

// Forward declarations
static void* mavlink_connect(const char* address);
static void mavlink_disconnect(void* conn);
static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size, void* msg_dict, size_t msg_dict_size);
static int mavlink_send_heartbeat(void* conn);
static int mavlink_send_set_position_target_local_ned(void* conn, uint32_t time_boot_ms, uint8_t coordinate_frame,
                                                       uint16_t type_mask, const vec3_t* pos, const vec3_t* vel,
                                                       const vec3_t* acc, double yaw, double yaw_rate);
static int mavlink_send_set_attitude_target(void* conn, uint32_t time_boot_ms, uint16_t type_mask,
                                            const quaternion_t* q, double body_roll_rate, double body_pitch_rate,
                                            double body_yaw_rate, double thrust);
static int mavlink_send_command_long(void* conn, uint16_t command, float param1, float param2, float param3,
                                     float param4, float param5, float param6, float param7);
static int mavlink_wait_heartbeat(void* conn, int timeout_ms);
static const char* mavlink_get_message_name_by_id(uint32_t msgid);

// Serialization helper (simplified - in production use proper serialization like msgpack or protobuf)
static size_t flight_data_serialize(const flight_data_t* fd, void* buffer, size_t buffer_size);
static int flight_data_deserialize(flight_data_t* fd, const void* buffer, size_t buffer_size);

// Command deserialization helpers
static int deserialize_attitude_cmd(const void* data, size_t data_len, quaternion_t* quat, vec3_t* rpy_rate, 
                                     double* thrust, bool* is_rate);
static int deserialize_vel_cmd(const void* data, size_t data_len, vec3_t* vel, double* yaw, double* yaw_rate);

// Initialize hardware adapter
int hardware_adapter_init(hardware_adapter_t* adapter, const char* log_dir) {
    if (adapter == NULL) {
        return -1;
    }
    
    // Note: adapter should be zero-initialized by caller (using calloc)
    // Initialize log directory
    if (log_dir != NULL) {
        adapter->log_dir = strdup(log_dir);
        if (adapter->log_dir == NULL) {
            fprintf(stderr, "Failed to allocate memory for log_dir\n");
            return -1;
        }
    } else {
        adapter->log_dir = strdup("./logs/");
        if (adapter->log_dir == NULL) {
            fprintf(stderr, "Failed to allocate memory for log_dir\n");
            return -1;
        }
    }
    
    // Initialize mutex
    if (pthread_mutex_init(&adapter->data_lock, NULL) != 0) {
        fprintf(stderr, "Failed to initialize data mutex\n");
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Initialize filters
    lpf_init(&adapter->vertical_speed_filter, 0.1, false, LPF_TYPE_FIRST_ORDER);
    lpf_init(&adapter->altitude_filter, 0.3, false, LPF_TYPE_FIRST_ORDER);
    adapter->prev_alt_m = -1.0;  // Invalid value to indicate not initialized
    adapter->alt_vel_count = 0;
    adapter->prev_vel_vertical = 0.0;
    adapter->prev_alt_ts = 0.0;
    
    // Initialize flight data
    flight_data_init(&adapter->current_data);
    
    // Initialize ZMQ
    zmq_wrapper_init();
    
    // Initialize mavlink
    // Use server mode (udp:14540) to listen on port 14540, matching the working UDP example
    // This allows receiving messages from the MAVLink server which sends to port 14540
    adapter->mavlink_address = strdup("udp:14540");
    if (adapter->mavlink_address == NULL) {
        fprintf(stderr, "Failed to allocate memory for mavlink_address\n");
        pthread_mutex_destroy(&adapter->data_lock);
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    if (hardware_adapter_init_mavlink(adapter) != 0) {
        fprintf(stderr, "Failed to initialize mavlink connection\n");
        adapter->init_success = false;
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
        pthread_mutex_destroy(&adapter->data_lock);
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Create ZMQ publisher socket
    adapter->pub_socket = zmq_publisher_create(TOPIC_MAVLINK_PORT);
    if (adapter->pub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ publisher socket\n");
        adapter->init_success = false;
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
        pthread_mutex_destroy(&adapter->data_lock);
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Wait a bit for publisher to bind
    usleep(200000);  // 200ms
    
    // Create ZMQ subscriber socket
    adapter->sub_socket = zmq_subscriber_create(TOPIC_GUIDANCE_CMD_PORT);
    if (adapter->sub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ subscriber socket\n");
        adapter->init_success = false;
        zmq_publisher_destroy(adapter->pub_socket);
        adapter->pub_socket = NULL;
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
        pthread_mutex_destroy(&adapter->data_lock);
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Subscribe to all command topics
    printf("Hardware_adapter: Subscribing to command topics...\n");
    
    // CRITICAL: For multipart messages, subscribe to empty string to receive ALL messages
    // The receive function will handle topic detection and filtering
    // This avoids issues with ZMQ subscription filters potentially stripping topic frames
    printf("Hardware_adapter: Subscribing to empty string to receive all messages (topic filtering done in receive function)...\n");
    if (zmq_subscriber_subscribe(adapter->sub_socket, "") != 0) {
        fprintf(stderr, "Failed to subscribe to empty string (all messages)\n");
        return -1;
    }
    printf("Hardware_adapter: Successfully subscribed to all messages\n");
    
    // Wait for subscriber to connect and for publisher to be ready
    // Note: In ZMQ PUB/SUB, messages sent before subscriber connects are lost
    usleep(500000);  // 500ms to ensure connection is established
    
    // Initialize control flags
    adapter->mavlink_connected_to_usb = false;
    adapter->disable_offboard_control = false;
    adapter->offboard_control_enabled = false;
    adapter->running = true;
    adapter->init_success = true;
    
    // Start threads
    if (pthread_create(&adapter->command_thread, NULL, hardware_adapter_command_thread_func, adapter) != 0) {
        fprintf(stderr, "Failed to create command thread\n");
        adapter->init_success = false;
        return -1;
    }
    
    if (pthread_create(&adapter->data_thread, NULL, hardware_adapter_data_thread_func, adapter) != 0) {
        fprintf(stderr, "Failed to create data thread\n");
        adapter->running = false;
        pthread_join(adapter->command_thread, NULL);
        adapter->init_success = false;
        return -1;
    }
    
    printf("Hardware adapter threads started successfully\n");
    printf("Publishing to topic: %s on port: %d\n", TOPIC_MAVLINK_FLIGHT_DATA, TOPIC_MAVLINK_PORT);
    
    return 0;
}

void hardware_adapter_cleanup(hardware_adapter_t* adapter) {
    if (adapter == NULL) {
        return;
    }
    
    // Stop threads
    hardware_adapter_stop(adapter);
    
    // Close ZMQ sockets
    if (adapter->pub_socket != NULL) {
        zmq_publisher_destroy(adapter->pub_socket);
        adapter->pub_socket = NULL;
    }
    if (adapter->sub_socket != NULL) {
        zmq_subscriber_destroy(adapter->sub_socket);
        adapter->sub_socket = NULL;
    }
    
    // Disconnect mavlink
    if (adapter->mavlink_connection != NULL) {
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
    }
    
    // Cleanup flight data
    flight_data_cleanup(&adapter->current_data);
    
    // Destroy mutex
    pthread_mutex_destroy(&adapter->data_lock);
    
    // Free strings
    if (adapter->log_dir != NULL) {
        free(adapter->log_dir);
        adapter->log_dir = NULL;
    }
    if (adapter->mavlink_address != NULL) {
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
    }
    
    zmq_wrapper_cleanup();
}

bool hardware_adapter_init_succeeded(hardware_adapter_t* adapter) {
    return adapter != NULL && adapter->init_success;
}

void hardware_adapter_stop(hardware_adapter_t* adapter) {
    if (adapter == NULL) {
        return;
    }
    
    adapter->running = false;
    
    // Wait for threads to finish
    if (adapter->command_thread != 0) {
        pthread_join(adapter->command_thread, NULL);
    }
    if (adapter->data_thread != 0) {
        pthread_join(adapter->data_thread, NULL);
    }
}

// Initialize mavlink connection
int hardware_adapter_init_mavlink(hardware_adapter_t* adapter) {
    adapter->mavlink_connection = (void*)mavlink_connect(adapter->mavlink_address);
    if (adapter->mavlink_connection == NULL) {
        return -1;
    }
    
    // Send heartbeat (try a few times)
    // Note: We don't require an immediate response - the MAVLink server might not be running yet
    // The connection will work once the server starts sending messages
    int count = 0;
    while (count < 3) {
        count++;
        mavlink_send_heartbeat(adapter->mavlink_connection);
        // Try to wait for heartbeat, but don't fail if we don't get one
        if (mavlink_wait_heartbeat(adapter->mavlink_connection, 2000) == 0) {
            printf("MAVLink heartbeat received\n");
            return 0;
        }
    }
    
    // Connection established, but no heartbeat yet - this is OK
    // The heartbeat will come when the MAVLink server is running
    printf("MAVLink connection established (waiting for heartbeat from server)\n");
    return 0;
}

// Listen to mavlink messages
void hardware_adapter_listener_to_mavlink(hardware_adapter_t* adapter, bool blocking, double timeout, 
                                          bool use_lock, bool apply_filter) {
    char msg_type[64];
    char msg_dict[1024];  // Simplified - in production use proper message structure
    
    int timeout_ms = blocking ? (int)(timeout * 1000) : 0;
    int result = mavlink_receive_message(adapter->mavlink_connection, msg_type, sizeof(msg_type),
                                        msg_dict, sizeof(msg_dict));
    
    if (result < 0) {
        return;  // No message or error
    }
    
    // Parse message
    if (use_lock) {
        pthread_mutex_lock(&adapter->data_lock);
        hardware_adapter_parse(adapter, msg_type, msg_dict);
        if (apply_filter) {
            hardware_adapter_filter_data(adapter, &adapter->current_data);
        }
        pthread_mutex_unlock(&adapter->data_lock);
    } else {
        hardware_adapter_parse(adapter, msg_type, msg_dict);
        if (apply_filter) {
            hardware_adapter_filter_data(adapter, &adapter->current_data);
        }
    }
}

// Filter data
void hardware_adapter_filter_data(hardware_adapter_t* adapter, flight_data_t* current_data) {
    double vel_vertical = 0.0;
    
    if (adapter->prev_alt_m >= 0.0) {  // Valid previous altitude
        double dt = current_data->altitude_m.timestamp - adapter->prev_alt_ts;
        if (dt != 0.0) {
            current_data->altitude_m.relative = lpf_step(&adapter->altitude_filter, 
                                                          current_data->altitude_m.relative);
            double d_alt = current_data->altitude_m.relative - adapter->prev_alt_m;
            vel_vertical = d_alt / dt;
            
            if (fabs(vel_vertical - adapter->prev_vel_vertical) > MAX_VERTICAL_VEL_JUMP_M_S) {
                if (vel_vertical - adapter->prev_vel_vertical > 0) {
                    vel_vertical = adapter->prev_vel_vertical + MAX_VERTICAL_VEL_JUMP_M_S;
                } else {
                    vel_vertical = adapter->prev_vel_vertical - MAX_VERTICAL_VEL_JUMP_M_S;
                }
            }
            
            current_data->altitude_m.vertical_speed_estimate = lpf_step(&adapter->vertical_speed_filter, 
                                                                        vel_vertical);
            adapter->prev_vel_vertical = current_data->altitude_m.vertical_speed_estimate;
            adapter->prev_alt_m = current_data->altitude_m.relative;
            adapter->prev_alt_ts = current_data->altitude_m.timestamp;
        }
    } else {
        adapter->prev_alt_m = current_data->altitude_m.relative;
        adapter->prev_alt_ts = current_data->altitude_m.timestamp;
    }
}

// Parse mavlink message
void hardware_adapter_parse(hardware_adapter_t* adapter, const char* msg_type, void* msg_dict) {
    if (adapter == NULL || msg_type == NULL || msg_dict == NULL) {
        return;
    }
    
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    adapter->current_data.local_ts = ts.tv_sec + ts.tv_nsec / 1e9;
    
    mavlink_message_t* msg = (mavlink_message_t*)msg_dict;
    
    // Parse message based on type
    if (strcmp(msg_type, "HEARTBEAT") == 0) {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        
        // Ignore mode updates if custom_mode is 0 (invalid/uninitialized)
        // Only update mode if we receive a valid non-zero mode value
        if (heartbeat.custom_mode != 0) {
            // Debug: Track mode changes
            static int prev_custom_mode_id = -1;
            static int mode_change_count = 0;
            static double last_mode_debug_time = 0;
            double current_time = adapter->current_data.local_ts;
            
            if (heartbeat.custom_mode != prev_custom_mode_id) {
                mode_change_count++;
                // Print mode change info (limit to avoid spam)
                if (current_time - last_mode_debug_time > 1.0 || mode_change_count <= 5) {
                    printf("hardware_adapter: Mode change #%d: %d -> %d (OFFBOARD=393216, HOLD=50593792)\n",
                           mode_change_count, prev_custom_mode_id, heartbeat.custom_mode);
                    last_mode_debug_time = current_time;
                }
                prev_custom_mode_id = heartbeat.custom_mode;
            }
            
            adapter->current_data.custom_mode_id = heartbeat.custom_mode;
            adapter->current_data.gathered.custom_mode_id = true;
        }
        // Note: mode string would need to be converted from custom_mode_id
    }
    else if (strcmp(msg_type, "ATTITUDE_QUATERNION") == 0) {
        mavlink_attitude_quaternion_t att_quat;
        mavlink_msg_attitude_quaternion_decode(msg, &att_quat);
        
        quaternion_init(&adapter->current_data.quat_ned_bodyfrd,
                       att_quat.q2, att_quat.q3, att_quat.q4, att_quat.q1);
        adapter->current_data.timestamp = att_quat.time_boot_ms;
        adapter->current_data.rpy_rates.data[0] = att_quat.rollspeed;
        adapter->current_data.rpy_rates.data[1] = att_quat.pitchspeed;
        adapter->current_data.rpy_rates.data[2] = att_quat.yawspeed;
        adapter->current_data.gathered.quat_ned_bodyfrd = true;
        adapter->current_data.gathered.rpy_rates = true;
    }
    else if (strcmp(msg_type, "ATTITUDE") == 0) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        
        adapter->current_data.timestamp = attitude.time_boot_ms;
        adapter->current_data.rpy.data[0] = attitude.roll;
        adapter->current_data.rpy.data[1] = attitude.pitch;
        adapter->current_data.rpy.data[2] = attitude.yaw;
        adapter->current_data.rpy_rates.data[0] = attitude.rollspeed;
        adapter->current_data.rpy_rates.data[1] = attitude.pitchspeed;
        adapter->current_data.rpy_rates.data[2] = attitude.yawspeed;
        adapter->current_data.gathered.rpy = true;
        adapter->current_data.gathered.rpy_rates = true;
    }
    else if (strcmp(msg_type, "LOCAL_POSITION_NED") == 0) {
        mavlink_local_position_ned_t local_pos;
        mavlink_msg_local_position_ned_decode(msg, &local_pos);
        
        adapter->current_data.timestamp = local_pos.time_boot_ms;
        adapter->current_data.pos_ned_m.ned.data[0] = local_pos.x;
        adapter->current_data.pos_ned_m.ned.data[1] = local_pos.y;
        adapter->current_data.pos_ned_m.ned.data[2] = local_pos.z;
        adapter->current_data.pos_ned_m.vel_ned.data[0] = local_pos.vx;
        adapter->current_data.pos_ned_m.vel_ned.data[1] = local_pos.vy;
        adapter->current_data.pos_ned_m.vel_ned.data[2] = local_pos.vz;
        adapter->current_data.pos_ned_m.timestamp = local_pos.time_boot_ms;
        adapter->current_data.gathered.pos_ned_m = true;
        adapter->current_data.gathered.vel_ned_m = true;
    }
    else if (strcmp(msg_type, "GLOBAL_POSITION_INT") == 0) {
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(msg, &global_pos);
        
        adapter->current_data.timestamp = global_pos.time_boot_ms;
        adapter->current_data.filt_pos_lla_deg.lla.data[0] = global_pos.lat / 1e7;
        adapter->current_data.filt_pos_lla_deg.lla.data[1] = global_pos.lon / 1e7;
        adapter->current_data.filt_pos_lla_deg.lla.data[2] = global_pos.alt / 1e3;
        adapter->current_data.relative_m = global_pos.relative_alt / 1000.0;
        adapter->current_data.heading = DEG2RAD(global_pos.hdg / 100.0);
        adapter->current_data.pos_ned_m.vel_ned.data[0] = global_pos.vx / 100.0;
        adapter->current_data.pos_ned_m.vel_ned.data[1] = global_pos.vy / 100.0;
        adapter->current_data.pos_ned_m.vel_ned.data[2] = global_pos.vz / 100.0;
        adapter->current_data.gathered.pos_ned_m = true;
        adapter->current_data.gathered.vel_ned_m = true;
    }
    else if (strcmp(msg_type, "HIGHRES_IMU") == 0) {
        mavlink_highres_imu_t imu;
        mavlink_msg_highres_imu_decode(msg, &imu);
        
        adapter->current_data.timestamp = imu.time_usec / 1000.0;
        adapter->current_data.imu_ned.accel.data[0] = imu.xacc;
        adapter->current_data.imu_ned.accel.data[1] = imu.yacc;
        adapter->current_data.imu_ned.accel.data[2] = imu.zacc;
        adapter->current_data.imu_ned.gyro.data[0] = imu.xgyro;
        adapter->current_data.imu_ned.gyro.data[1] = imu.ygyro;
        adapter->current_data.imu_ned.gyro.data[2] = imu.zgyro;
        adapter->current_data.imu_ned.timestamp = adapter->current_data.timestamp;
        adapter->current_data.absolute_press_hpa = imu.abs_pressure;
        adapter->current_data.differential_press_hpa = imu.diff_pressure;
        adapter->current_data.pressure = imu.pressure_alt;
        adapter->current_data.temperature = imu.temperature;
        adapter->current_data.gathered.imu_ned = true;
        adapter->current_data.gathered.absolute_press_hpa = true;
        adapter->current_data.gathered.differential_press_hpa = true;
        adapter->current_data.gathered.pressure = true;
        adapter->current_data.gathered.temperature = true;
    }
    else if (strcmp(msg_type, "ALTITUDE") == 0) {
        mavlink_altitude_t altitude;
        mavlink_msg_altitude_decode(msg, &altitude);
        
        adapter->current_data.timestamp = altitude.time_usec / 1000.0;
        adapter->current_data.relative_m = altitude.altitude_relative;
        adapter->current_data.amsl_m = altitude.altitude_amsl;
        adapter->current_data.local_m = altitude.altitude_local;
        adapter->current_data.monotonic_m = altitude.altitude_monotonic;
        adapter->current_data.terrain_m = altitude.altitude_terrain;
        adapter->current_data.bottom_clearance_m = altitude.bottom_clearance;
        adapter->current_data.altitude_m.relative = altitude.altitude_relative;
        adapter->current_data.altitude_m.amsl = altitude.altitude_amsl;
        adapter->current_data.altitude_m.timestamp = adapter->current_data.timestamp;
        adapter->current_data.gathered.relative_m = true;
        adapter->current_data.gathered.amsl_m = true;
        adapter->current_data.gathered.local_m = true;
        adapter->current_data.gathered.monotonic_m = true;
        adapter->current_data.gathered.terrain_m = true;
        adapter->current_data.gathered.bottom_clearance_m = true;
    }
    else if (strcmp(msg_type, "VFR_HUD") == 0) {
        mavlink_vfr_hud_t vfr_hud;
        mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
        
        adapter->current_data.altitude_m.relative = vfr_hud.alt;
        adapter->current_data.heading = DEG2RAD(vfr_hud.heading);
        adapter->current_data.throttle = vfr_hud.throttle;
    }
}

// Send setpoint
void hardware_adapter_send_setpoint(hardware_adapter_t* adapter, const vec3_t* pos, const vec3_t* vel,
                                    const vec3_t* acc, double yaw, double yaw_rate) {
    if (adapter->mavlink_connection == NULL) {
        return;
    }
    
    uint32_t time_boot_ms = 0;
    uint8_t coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
    uint16_t type_mask = 0;
    
    vec3_t pos_vec, vel_vec, acc_vec;
    if (pos == NULL) {
        type_mask |= 0x07;  // Ignore position
        vec3_zero(&pos_vec);
    } else {
        vec3_copy(&pos_vec, pos);
    }
    
    if (vel == NULL) {
        type_mask |= 0x38;  // Ignore velocity
        vec3_zero(&vel_vec);
    } else {
        vec3_copy(&vel_vec, vel);
    }
    
    if (acc == NULL) {
        type_mask |= 0x1C0;  // Ignore acceleration
        vec3_zero(&acc_vec);
    } else {
        vec3_copy(&acc_vec, acc);
    }
    
    if (isnan(yaw)) {
        type_mask |= 0x400;  // Ignore yaw
        yaw = 0.0;
    }
    
    if (isnan(yaw_rate)) {
        type_mask |= 0x800;  // Ignore yaw rate
        yaw_rate = 0.0;
    }
    
    mavlink_send_set_position_target_local_ned(adapter->mavlink_connection, time_boot_ms, coordinate_frame,
                                               type_mask, &pos_vec, &vel_vec, &acc_vec, yaw, yaw_rate);
}

// Send goal attitude
void hardware_adapter_send_goal_attitude(hardware_adapter_t* adapter, double goal_thrust,
                                         const quaternion_t* goal_attitude, const rate_cmd_t* rates) {
    if (!adapter->offboard_control_enabled || adapter->mavlink_connection == NULL) {
        return;
    }
    
    uint32_t time_boot_ms = (uint32_t)(time(NULL) * 1000);
    uint16_t type_mask = 0;
    
    quaternion_t q;
    double body_roll_rate = 0.0, body_pitch_rate = 0.0, body_yaw_rate = 0.0;
    
    if (goal_attitude == NULL) {
        type_mask |= 0x01;  // Ignore attitude
        quaternion_init(&q, 0.0, 0.0, 0.0, 1.0);
    } else {
        quaternion_copy(&q, goal_attitude);
    }
    
    if (rates == NULL) {
        type_mask |= 0x0E;  // Ignore rates
    } else {
        body_roll_rate = rates->rpydot.data[0];
        body_pitch_rate = rates->rpydot.data[1];
        body_yaw_rate = rates->rpydot.data[2];
    }
    
    mavlink_send_set_attitude_target(adapter->mavlink_connection, time_boot_ms, type_mask,
                                     &q, body_roll_rate, body_pitch_rate, body_yaw_rate, goal_thrust);
}

// Send offboard command
void hardware_adapter_send_offboard_cmd(hardware_adapter_t* adapter) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // Send command to set mode to OFFBOARD
    // Mode ID for OFFBOARD (PX4) is typically 6
    mavlink_send_command_long(adapter->mavlink_connection, 176, 1.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Arm vehicle
void hardware_adapter_arm(hardware_adapter_t* adapter) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // MAV_CMD_COMPONENT_ARM_DISARM with param1=1 to arm
    mavlink_send_command_long(adapter->mavlink_connection, 400, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Send takeoff command
void hardware_adapter_send_takeoff_cmd(hardware_adapter_t* adapter, double takeoff_altitude) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // This is a simplified version - in production, get current altitude first
    // and send proper takeoff command
    mavlink_send_command_long(adapter->mavlink_connection, 22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, takeoff_altitude);
}

// Send land command
void hardware_adapter_send_land_cmd(hardware_adapter_t* adapter) {
    mavlink_send_command_long(adapter->mavlink_connection, 21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Command thread function
void* hardware_adapter_command_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    char topic_buffer[256];
    char data_buffer[4096];
    struct timespec ts;
    double last_command_time = 0.0;
    double last_warning_time = 0.0;
    bool has_received_command = false;
    const double WARNING_INTERVAL = 5.0;  // Print warning every 5 seconds if no commands
    static int total_receive_attempts = 0;
    static int successful_receives = 0;
    static int unmatched_messages = 0;
    
    printf("Hardware_adapter: Command thread started, waiting for commands...\n");
    printf("Hardware_adapter: Subscribed to topics on port %d: %s, %s, %s, %s, %s\n",
           TOPIC_GUIDANCE_CMD_PORT,
           TOPIC_GUIDANCE_CMD_ATTITUDE, TOPIC_GUIDANCE_CMD_VEL_NED,
           TOPIC_GUIDANCE_CMD_VEL_BODY, TOPIC_GUIDANCE_CMD_ACC, TOPIC_GUIDANCE_CMD_ARM);
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double start_time = ts.tv_sec + ts.tv_nsec / 1e9;
    last_command_time = start_time;
    last_warning_time = start_time;  // Initialize to start time so first warning happens after 5 seconds
    
    // Test connection by trying to receive a message (blocking for 2 seconds)
    printf("Hardware_adapter: Testing connection - waiting up to 2 seconds for a message...\n");
    int test_len = zmq_subscriber_receive(adapter->sub_socket, topic_buffer, sizeof(topic_buffer),
                                         data_buffer, sizeof(data_buffer), 2000);
    if (test_len >= 0) {
        printf("Hardware_adapter: SUCCESS - Received test message! Topic: %s, len: %d\n", topic_buffer, test_len);
    } else if (test_len == -2) {
        printf("Hardware_adapter: Received message but topic didn't match (this is OK, means connection works)\n");
    } else {
        printf("Hardware_adapter: WARNING - No message received in 2 seconds. Connection may not be established.\n");
        printf("Hardware_adapter: Check that system_manager is running and publishing on port %d\n", TOPIC_GUIDANCE_CMD_PORT);
    }
    
    while (adapter->running) {
        int data_len = zmq_subscriber_receive(adapter->sub_socket, topic_buffer, sizeof(topic_buffer),
                                             data_buffer, sizeof(data_buffer), 100);
        
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double current_time = ts.tv_sec + ts.tv_nsec / 1e9;
        
        total_receive_attempts++;
        
        if (data_len == -2) {
            // Message received but topic didn't match
            unmatched_messages++;
            usleep(1000);
            continue;
        }
        
        if (data_len < 0) {
            // Check if we should print a warning about no commands
            // Print every 5 seconds if no commands received (whether we've received one before or not)
            double time_since_last_command = current_time - last_command_time;
            double time_since_last_warning = current_time - last_warning_time;
            
            if (time_since_last_command >= WARNING_INTERVAL && time_since_last_warning >= WARNING_INTERVAL) {
                if (has_received_command) {
                    printf("Hardware_adapter: WARNING - No guidance commands received for %.1f seconds (check if system_manager is sending commands)\n",
                           time_since_last_command);
                } else {
                    printf("Hardware_adapter: WARNING - No guidance commands received since startup (%.1f seconds). Waiting for commands on port %d, topics: %s, %s, %s, %s, %s\n",
                           time_since_last_command, TOPIC_GUIDANCE_CMD_PORT,
                           TOPIC_GUIDANCE_CMD_ATTITUDE, TOPIC_GUIDANCE_CMD_VEL_NED,
                           TOPIC_GUIDANCE_CMD_VEL_BODY, TOPIC_GUIDANCE_CMD_ACC, TOPIC_GUIDANCE_CMD_ARM);
                    printf("Hardware_adapter: DEBUG - Total receive attempts: %d, Successful: %d, Unmatched: %d\n",
                           total_receive_attempts, successful_receives, unmatched_messages);
                }
                last_warning_time = current_time;
            }
            usleep(1000);  // 1ms
            continue;
        }
        
        successful_receives++;
        
        // Update last command time when we receive a command
        last_command_time = current_time;
        if (!has_received_command) {
            printf("Hardware_adapter: First command received! Topic: %s, data_len: %d\n", topic_buffer, data_len);
        }
        has_received_command = true;
        
        // Process command based on topic
        if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ATTITUDE) == 0) {
            quaternion_t quat;
            vec3_t rpy_rate;
            double thrust;
            bool is_rate;
            
            if (deserialize_attitude_cmd(data_buffer, data_len, &quat, &rpy_rate, &thrust, &is_rate) == 0) {
                if (is_rate) {
                    rate_cmd_t rate_cmd;
                    rate_cmd_init(&rate_cmd, &rpy_rate);
                    hardware_adapter_send_goal_attitude(adapter, thrust, NULL, &rate_cmd);
                } else {
                    hardware_adapter_send_goal_attitude(adapter, thrust, &quat, NULL);
                }
            } else {
                static int deserialize_error_count = 0;
                if (deserialize_error_count++ % 100 == 0) {
                    printf("Hardware_adapter: Failed to deserialize attitude command (data_len=%d)\n", data_len);
                }
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_NED) == 0) {
            vec3_t vel;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel, &yaw, &yaw_rate) == 0) {
                double yaw_cmd = isnan(yaw) ? NAN : yaw;
                double yaw_rate_cmd = isnan(yaw_rate) ? NAN : yaw_rate;
                hardware_adapter_send_setpoint(adapter, NULL, &vel, NULL, yaw_cmd, yaw_rate_cmd);
                
                // Debug: Print first few successful commands
                static int vel_cmd_count = 0;
                if (vel_cmd_count++ < 5) {
                    printf("Hardware_adapter: Received velocity command: vel=[%.3f, %.3f, %.3f], yaw=%.3f, yaw_rate=%.3f\n",
                           vel.data[0], vel.data[1], vel.data[2], yaw_cmd, yaw_rate_cmd);
                }
            } else {
                static int deserialize_error_count = 0;
                if (deserialize_error_count++ % 100 == 0) {
                    printf("Hardware_adapter: Failed to deserialize velocity command (data_len=%d, first bytes: %02x %02x %02x %02x)\n", 
                           data_len, 
                           data_len > 0 ? ((uint8_t*)data_buffer)[0] : 0,
                           data_len > 1 ? ((uint8_t*)data_buffer)[1] : 0,
                           data_len > 2 ? ((uint8_t*)data_buffer)[2] : 0,
                           data_len > 3 ? ((uint8_t*)data_buffer)[3] : 0);
                }
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_BODY) == 0) {
            vec3_t vel_body, vel_ned;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel_body, &yaw, &yaw_rate) == 0) {
                // Rotate body velocity to NED frame
                pthread_mutex_lock(&adapter->data_lock);
                if (adapter->current_data.gathered.quat_ned_bodyfrd) {
                    vel_ned = quaternion_rotate_vec(&adapter->current_data.quat_ned_bodyfrd, &vel_body);
                } else {
                    vec3_copy(&vel_ned, &vel_body);  // Fallback if quaternion not available
                }
                pthread_mutex_unlock(&adapter->data_lock);
                
                double yaw_cmd = isnan(yaw) ? NAN : yaw;
                double yaw_rate_cmd = isnan(yaw_rate) ? NAN : yaw_rate;
                hardware_adapter_send_setpoint(adapter, NULL, &vel_ned, NULL, yaw_cmd, yaw_rate_cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ACC) == 0) {
            vec3_t acc;
            double yaw, yaw_rate;
            
            // Similar to vel command but for acceleration
            // Implementation would deserialize acc command and send setpoint
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ARM) == 0) {
            hardware_adapter_arm(adapter);
        }
    }
    
    return NULL;
}

// Data thread function
void* hardware_adapter_data_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    struct timespec ts;
    double out_time = 0.0;
    double print_time = 0.0;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double start_time = ts.tv_sec + ts.tv_nsec / 1e9;
    out_time = start_time + PUBLISH_DT;
    print_time = start_time + 1.0;
    
    while (adapter->running) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double current_time = ts.tv_sec + ts.tv_nsec / 1e9;
        
        // Process mavlink messages (non-blocking, will return immediately if no message)
        hardware_adapter_listener_to_mavlink(adapter, false, 0.0, true, true);
        
        // Publish data at specified frequency
        if (current_time >= out_time) {
            out_time = current_time + PUBLISH_DT;
            
            pthread_mutex_lock(&adapter->data_lock);
            adapter->current_data.local_ts = current_time;
            
            // Serialize flight data
            char serialized_data[8192];
            size_t data_len = flight_data_serialize(&adapter->current_data, serialized_data, sizeof(serialized_data));
            
            if (data_len > 0) {
                zmq_publisher_send(adapter->pub_socket, TOPIC_MAVLINK_FLIGHT_DATA, serialized_data, data_len);
                adapter->current_data.message_count++;
            }
            pthread_mutex_unlock(&adapter->data_lock);
        }
        
        // Print status at lower frequency
        if (current_time >= print_time) {
            print_time = current_time + 1.0;
            pthread_mutex_lock(&adapter->data_lock);
            if (adapter->current_data.timestamp > 0) {
                printf("MAVLink timestamp: %.3f ms, local_ts: %.3f s\n", 
                       adapter->current_data.timestamp, adapter->current_data.local_ts);
            } else {
                printf("Waiting for MAVLink messages... (local_ts: %.3f s)\n", 
                       adapter->current_data.local_ts);
            }
            pthread_mutex_unlock(&adapter->data_lock);
        }
        
        usleep(100);  // 0.1ms
    }
    
    return NULL;
}

// Parse MAVLink address string (format: "udp:IP:PORT" or "udp:PORT")
// Returns: 0 on success, -1 on error
// Sets is_server_mode: true if just port (listen mode), false if IP:PORT (client mode)
static int parse_mavlink_address(const char* address, char* ip, size_t ip_size, int* port, bool* is_server_mode) {
    if (address == NULL || ip == NULL || port == NULL || is_server_mode == NULL) {
        return -1;
    }
    
    // Default values
    strncpy(ip, "127.0.0.1", ip_size);
    *port = 14540;
    *is_server_mode = false;
    
    // Check if it starts with "udp:"
    if (strncmp(address, "udp:", 4) != 0) {
        fprintf(stderr, "Invalid MAVLink address format, expected 'udp:IP:PORT' or 'udp:PORT'\n");
        return -1;
    }
    
    const char* addr_part = address + 4;  // Skip "udp:"
    
    // Check if it's just a port number (e.g., "udp:14540")
    char* colon = strchr(addr_part, ':');
    if (colon == NULL) {
        // No colon, assume it's just a port - server mode (listen on this port)
        *port = atoi(addr_part);
        if (*port <= 0 || *port > 65535) {
            fprintf(stderr, "Invalid port number: %s\n", addr_part);
            return -1;
        }
        *is_server_mode = true;
    } else {
        // Has colon, parse IP and port - client mode (connect to this address)
        size_t ip_len = colon - addr_part;
        if (ip_len >= ip_size) {
            fprintf(stderr, "IP address too long\n");
            return -1;
        }
        strncpy(ip, addr_part, ip_len);
        ip[ip_len] = '\0';
        
        const char* port_str = colon + 1;
        *port = atoi(port_str);
        if (*port <= 0 || *port > 65535) {
            fprintf(stderr, "Invalid port number: %s\n", port_str);
            return -1;
        }
        *is_server_mode = false;
    }
    
    return 0;
}

// Create UDP socket and connect to MAVLink endpoint
static void* mavlink_connect(const char* address) {
    char ip[64];
    int port;
    bool is_server_mode;
    
    if (parse_mavlink_address(address, ip, sizeof(ip), &port, &is_server_mode) != 0) {
        fprintf(stderr, "Failed to parse MAVLink address: %s\n", address);
        return NULL;
    }
    
    mavlink_connection_t* conn = (mavlink_connection_t*)calloc(1, sizeof(mavlink_connection_t));
    if (conn == NULL) {
        fprintf(stderr, "Failed to allocate MAVLink connection\n");
        return NULL;
    }
    
    // Create UDP socket
    conn->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn->fd < 0) {
        fprintf(stderr, "Failed to create UDP socket: %s\n", strerror(errno));
        free(conn);
        return NULL;
    }
    
    // Enable SO_REUSEADDR to allow binding even if port is in use
    int reuse = 1;
    if (setsockopt(conn->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        fprintf(stderr, "Warning: Failed to set SO_REUSEADDR: %s\n", strerror(errno));
    }
    
    // Set socket to non-blocking
    int flags = fcntl(conn->fd, F_GETFL, 0);
    if (flags < 0 || fcntl(conn->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "Failed to set socket to non-blocking: %s\n", strerror(errno));
        close(conn->fd);
        free(conn);
        return NULL;
    }
    
    // Set receive timeout (like UDP example does)
    // This prevents being stuck in recvfrom for too long
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms timeout
    if (setsockopt(conn->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        fprintf(stderr, "Warning: Failed to set SO_RCVTIMEO: %s\n", strerror(errno));
    }
    
    // Set up local address
    // For UDP client mode (udp:IP:PORT): bind to any available port (server will respond to source port)
    // For UDP server mode (udp:PORT): bind to the specified port to listen (like UDP example)
    memset(&conn->local_addr, 0, sizeof(conn->local_addr));
    conn->local_addr.sin_family = AF_INET;
    conn->local_addr.sin_addr.s_addr = INADDR_ANY;
    if (is_server_mode) {
        conn->local_addr.sin_port = htons(port);  // Bind to specified port (server mode)
        printf("MAVLink server mode: listening on port %d\n", port);
    } else {
        conn->local_addr.sin_port = 0;  // Let system choose port (client mode)
    }
    
    // Set up remote address (where we send messages)
    // For server mode, remote address will be set from first received message
    // For client mode, set it to the specified IP:PORT
    memset(&conn->remote_addr, 0, sizeof(conn->remote_addr));
    if (!is_server_mode) {
        conn->remote_addr.sin_family = AF_INET;
        conn->remote_addr.sin_port = htons(port);
        if (inet_aton(ip, &conn->remote_addr.sin_addr) == 0) {
            fprintf(stderr, "Invalid IP address: %s\n", ip);
            close(conn->fd);
            free(conn);
            return NULL;
        }
    } else {
        // Server mode: remote address will be set from first received message
        conn->remote_addr.sin_family = AF_INET;
        conn->remote_addr.sin_addr.s_addr = INADDR_ANY;
        conn->remote_addr.sin_port = 0;
    }
    
    // Bind socket
    if (bind(conn->fd, (struct sockaddr*)&conn->local_addr, sizeof(conn->local_addr)) < 0) {
        fprintf(stderr, "Failed to bind UDP socket: %s\n", strerror(errno));
        close(conn->fd);
        free(conn);
        return NULL;
    }
    
    // Get the actual port we bound to
    socklen_t len = sizeof(conn->local_addr);
    if (getsockname(conn->fd, (struct sockaddr*)&conn->local_addr, &len) == 0) {
        printf("MAVLink UDP socket bound to local port %d\n", ntohs(conn->local_addr.sin_port));
    }
    
    // For UDP, we DON'T connect the socket
    // This allows us to receive from any source, not just the connected address
    conn->socket_connected = false;
    
    // Initialize MAVLink connection parameters
    conn->system_id = MAV_SYSTEM_ID;
    conn->component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    conn->target_system = 1;  // Will be updated when we receive heartbeat
    conn->target_component = MAV_COMP_ID_AUTOPILOT1;
    conn->connected = false;
    memset(&conn->status, 0, sizeof(conn->status));
    
    if (is_server_mode) {
        printf("MAVLink listening on port %d (server mode)\n", port);
    } else {
        printf("MAVLink client mode: will send to %s:%d\n", ip, port);
    }
    
    return (void*)conn;
}

static void mavlink_disconnect(void* conn) {
    if (conn == NULL) {
        return;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd >= 0) {
        close(mconn->fd);
        mconn->fd = -1;
    }
    free(conn);
}

static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size,
                                   void* msg_dict, size_t msg_dict_size) {
    if (conn == NULL || msg_type == NULL || msg_dict == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    
    // Receive UDP packet using recvfrom (works whether connected or not)
    // This allows us to receive from any source
    ssize_t recv_len = recvfrom(mconn->fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                                (struct sockaddr*)&src_addr, &addr_len);
    
    if (recv_len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return -1;  // No data available (normal for non-blocking socket)
        }
        // Only print errors for unexpected conditions (not connection refused for UDP)
        // Connection refused typically means nothing is listening, which is OK
        // Don't spam errors - only print once in a while
        static int error_count = 0;
        if (errno != ECONNREFUSED && (error_count++ % 1000 == 0)) {
            fprintf(stderr, "Error receiving MAVLink message (every 1000 attempts): %s\n", strerror(errno));
        }
        return -1;
    }
    
    // Debug: Print first message received and update remote address for server mode
    static bool first_message_received = false;
    if (recv_len > 0) {
        // Update remote address from first received message (for server mode)
        // This allows us to send responses back to the sender
        if (!first_message_received) {
            first_message_received = true;
            char src_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, INET_ADDRSTRLEN);
            printf("First UDP packet received from %s:%d (%zd bytes)\n", 
                   src_ip, ntohs(src_addr.sin_port), recv_len);
            
            // Update remote address so we can send responses
            memcpy(&mconn->remote_addr, &src_addr, sizeof(src_addr));
            printf("Updated remote address to %s:%d for sending responses\n",
                   src_ip, ntohs(src_addr.sin_port));
        }
    }
    
    if (recv_len < MAVLINK_NUM_HEADER_BYTES) {
        return -1;  // Too short to be a valid MAVLink message
    }
    
    // Parse MAVLink message character by character
    mavlink_message_t msg;
    mavlink_status_t status;
    bool message_received = false;
    
    for (ssize_t i = 0; i < recv_len; i++) {
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status);
        
        if (result == MAVLINK_FRAMING_OK) {
            // Valid message received
            mconn->status = status;
            message_received = true;
            
            // Update target system/component from heartbeat
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mconn->target_system = msg.sysid;
                mconn->target_component = msg.compid;
                mconn->connected = true;
            }
            
            // Convert message ID to string
            const char* msg_name = mavlink_get_message_name_by_id(msg.msgid);
            if (msg_name != NULL) {
                strncpy(msg_type, msg_name, msg_type_size - 1);
                msg_type[msg_type_size - 1] = '\0';
            } else {
                snprintf(msg_type, msg_type_size, "MSG_%u", (unsigned int)msg.msgid);
            }
            
            // Store message data
            if (msg_dict_size >= sizeof(mavlink_message_t)) {
                memcpy(msg_dict, &msg, sizeof(mavlink_message_t));
            }
            
            break;  // Found valid message, exit loop
        }
        // Continue parsing even if we get bad CRC/signature (might be partial message)
    }
    
    if (message_received) {
        return 0;  // Success
    }
    
    return -1;  // No valid message found
}

static int mavlink_send_heartbeat(void* conn) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    // In server mode, don't send until we've received a message (to know where to send)
    if (mconn->remote_addr.sin_addr.s_addr == INADDR_ANY || mconn->remote_addr.sin_port == 0) {
        return -1;  // No remote address set yet
    }
    
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    
    heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 3;
    
    mavlink_msg_heartbeat_encode(mconn->system_id, mconn->component_id, &msg, &heartbeat);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Always use sendto for UDP (works whether connected or not)
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->remote_addr, sizeof(mconn->remote_addr));
    
    // Debug: Print first heartbeat sent
    static bool first_heartbeat_sent = false;
    if (!first_heartbeat_sent && sent > 0) {
        first_heartbeat_sent = true;
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &mconn->remote_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
        printf("First MAVLink heartbeat sent to %s:%d (%d bytes)\n", 
               ip_str, ntohs(mconn->remote_addr.sin_port), len);
    }
    
    if (sent != len && sent >= 0) {
        fprintf(stderr, "Warning: Failed to send complete heartbeat: sent %zd of %d bytes\n", sent, len);
    }
    
    return (sent == len) ? 0 : -1;
}

static int mavlink_send_set_position_target_local_ned(void* conn, uint32_t time_boot_ms, uint8_t coordinate_frame,
                                                      uint16_t type_mask, const vec3_t* pos, const vec3_t* vel,
                                                      const vec3_t* acc, double yaw, double yaw_rate) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t setpoint;
    
    setpoint.time_boot_ms = time_boot_ms;
    setpoint.target_system = mconn->target_system;
    setpoint.target_component = mconn->target_component;
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
    
    // Always use sendto for UDP
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->remote_addr, sizeof(mconn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

static int mavlink_send_set_attitude_target(void* conn, uint32_t time_boot_ms, uint16_t type_mask,
                                            const quaternion_t* q, double body_roll_rate, double body_pitch_rate,
                                            double body_yaw_rate, double thrust) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    mavlink_message_t msg;
    mavlink_set_attitude_target_t attitude;
    
    attitude.time_boot_ms = time_boot_ms;
    attitude.target_system = mconn->target_system;
    attitude.target_component = mconn->target_component;
    attitude.type_mask = type_mask;
    
    if (q != NULL) {
        attitude.q[0] = (float)q->w;
        attitude.q[1] = (float)q->x;
        attitude.q[2] = (float)q->y;
        attitude.q[3] = (float)q->z;
    } else {
        attitude.q[0] = 1.0f;
        attitude.q[1] = 0.0f;
        attitude.q[2] = 0.0f;
        attitude.q[3] = 0.0f;
    }
    
    attitude.body_roll_rate = (float)body_roll_rate;
    attitude.body_pitch_rate = (float)body_pitch_rate;
    attitude.body_yaw_rate = (float)body_yaw_rate;
    attitude.thrust = (float)thrust;
    
    mavlink_msg_set_attitude_target_encode(mconn->system_id, mconn->component_id, &msg, &attitude);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Always use sendto for UDP
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->remote_addr, sizeof(mconn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

static int mavlink_send_command_long(void* conn, uint16_t command, float param1, float param2, float param3,
                                     float param4, float param5, float param6, float param7) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    
    cmd.target_system = mconn->target_system;
    cmd.target_component = mconn->target_component;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;
    
    mavlink_msg_command_long_encode(mconn->system_id, mconn->component_id, &msg, &cmd);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Always use sendto for UDP
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->remote_addr, sizeof(mconn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

static int mavlink_wait_heartbeat(void* conn, int timeout_ms) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    struct pollfd pfd;
    pfd.fd = mconn->fd;
    pfd.events = POLLIN;
    
    int poll_result = poll(&pfd, 1, timeout_ms);
    if (poll_result <= 0) {
        return -1;  // Timeout or error
    }
    
    // Try to receive a message
    char msg_type[64];
    char msg_dict[1024];
    int result = mavlink_receive_message(conn, msg_type, sizeof(msg_type), msg_dict, sizeof(msg_dict));
    
    if (result == 0 && strcmp(msg_type, "HEARTBEAT") == 0) {
        return 0;  // Heartbeat received
    }
    
    return -1;  // No heartbeat received
}

// Helper function to get message name from message ID
static const char* mavlink_get_message_name_by_id(uint32_t msgid) {
    // Map common MAVLink message IDs to their names
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            return "HEARTBEAT";
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            return "ATTITUDE_QUATERNION";
        case MAVLINK_MSG_ID_ATTITUDE:
            return "ATTITUDE";
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            return "LOCAL_POSITION_NED";
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            return "HIGHRES_IMU";
        case MAVLINK_MSG_ID_ALTITUDE:
            return "ALTITUDE";
        case MAVLINK_MSG_ID_VFR_HUD:
            return "VFR_HUD";
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            return "ATTITUDE_TARGET";
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            return "POSITION_TARGET_LOCAL_NED";
        case MAVLINK_MSG_ID_COMMAND_ACK:
            return "COMMAND_ACK";
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            return "DISTANCE_SENSOR";
        case MAVLINK_MSG_ID_OPTICAL_FLOW:
            return "OPTICAL_FLOW";
        default:
            return NULL;  // Unknown message ID
    }
}

// Serialization helpers - serialize to a format that Python can deserialize
// We serialize to a simple binary format that Python can read using struct module
// Format: All numeric fields in a fixed order, strings are serialized as length-prefixed UTF-8
static size_t flight_data_serialize(const flight_data_t* fd, void* buffer, size_t buffer_size) {
    if (fd == NULL || buffer == NULL) {
        return 0;
    }
    
    // Calculate required buffer size
    // We'll serialize all numeric fields, then handle strings separately
    // For now, use a simple approach: serialize numeric data, skip string pointers
    // Python will reconstruct the object from the numeric data
    
    size_t offset = 0;
    
    // Calculate size needed:
    // message_count (uint32_t = 4)
    // quat_ts, imu_ts, timestamp, local_ts, temperature, amsl_m, local_m, monotonic_m, 
    // relative_m, terrain_m, bottom_clearance_m, pressure, absolute_press_hpa, 
    // differential_press_hpa, signal_strength_percent, throttle, heading, groundspeed, 
    // current_thrust, quat_ts (double = 8 each, ~18 doubles = 144)
    // quaternion (4 doubles = 32), quat timestamp (1 double = 8)
    // altitude (4 doubles = 32)
    // imu_raw_frd: accel (3 doubles = 24), gyro (3 doubles = 24), timestamp (1 double = 8)
    // imu_ned: accel (3 doubles = 24), gyro (3 doubles = 24), timestamp (1 double = 8)
    // pos_ned_m: ned (3 doubles = 24), vel_ned (3 doubles = 24), timestamp (1 double = 8)
    // raw_pos_lla_deg: lla (3 doubles = 24), timestamp (1 double = 8)
    // filt_pos_lla_deg: lla (3 doubles = 24), timestamp (1 double = 8)
    // rpy_rates (3 doubles = 24), rpy (3 doubles = 24)
    // bools: is_armed, offboardMode, is_available, was_available_once, 
    //        is_gyrometer_calibration_ok, is_accelerometer_calibration_ok, 
    //        is_magnetometer_calibration_ok, in_air (8 bools = 8, but we'll pack as uint8)
    // custom_mode_id (uint32_t = 4), mode (enum, as uint32_t = 4)
    // gathered flags (as uint32_t bitfield = 4)
    // Total: ~500 bytes + some overhead
    
    // Use a simpler approach: serialize all fields in a fixed format
    // We'll use Python's struct module format: '<' for little-endian, then field types
    
    if (buffer_size < 1024) {  // Conservative estimate
        return 0;
    }
    
    uint8_t* buf = (uint8_t*)buffer;
    offset = 0;
    
    // Write magic number for format identification (4 bytes)
    uint32_t magic = 0x464C4947;  // "FLIG" in ASCII
    memcpy(buf + offset, &magic, 4);
    offset += 4;
    
    // Write version (uint32_t)
    uint32_t version = 1;
    memcpy(buf + offset, &version, 4);
    offset += 4;
    
    // Write message_count (uint32_t)
    memcpy(buf + offset, &fd->message_count, 4);
    offset += 4;
    
    // Write doubles (all in little-endian)
    double* dbl_fields[] = {
        &fd->quat_ts,
        &fd->imu_ts,
        &fd->timestamp,
        &fd->local_ts,
        &fd->temperature,
        &fd->amsl_m,
        &fd->local_m,
        &fd->monotonic_m,
        &fd->relative_m,
        &fd->terrain_m,
        &fd->bottom_clearance_m,
        &fd->pressure,
        &fd->absolute_press_hpa,
        &fd->differential_press_hpa,
        &fd->signal_strength_percent,
        &fd->throttle,
        &fd->heading,
        &fd->groundspeed,
        &fd->current_thrust,
        &fd->quat_ned_bodyfrd.x,
        &fd->quat_ned_bodyfrd.y,
        &fd->quat_ned_bodyfrd.z,
        &fd->quat_ned_bodyfrd.w,
        &fd->quat_ned_bodyfrd.timestamp,
        &fd->altitude_m.amsl,
        &fd->altitude_m.relative,
        &fd->altitude_m.vertical_speed_estimate,
        &fd->altitude_m.timestamp,
        &fd->imu_raw_frd.accel.data[0],
        &fd->imu_raw_frd.accel.data[1],
        &fd->imu_raw_frd.accel.data[2],
        &fd->imu_raw_frd.gyro.data[0],
        &fd->imu_raw_frd.gyro.data[1],
        &fd->imu_raw_frd.gyro.data[2],
        &fd->imu_raw_frd.timestamp,
        &fd->imu_ned.accel.data[0],
        &fd->imu_ned.accel.data[1],
        &fd->imu_ned.accel.data[2],
        &fd->imu_ned.gyro.data[0],
        &fd->imu_ned.gyro.data[1],
        &fd->imu_ned.gyro.data[2],
        &fd->imu_ned.timestamp,
        &fd->pos_ned_m.ned.data[0],
        &fd->pos_ned_m.ned.data[1],
        &fd->pos_ned_m.ned.data[2],
        &fd->pos_ned_m.vel_ned.data[0],
        &fd->pos_ned_m.vel_ned.data[1],
        &fd->pos_ned_m.vel_ned.data[2],
        &fd->pos_ned_m.timestamp,
        &fd->raw_pos_lla_deg.lla.data[0],
        &fd->raw_pos_lla_deg.lla.data[1],
        &fd->raw_pos_lla_deg.lla.data[2],
        &fd->raw_pos_lla_deg.timestamp,
        &fd->filt_pos_lla_deg.lla.data[0],
        &fd->filt_pos_lla_deg.lla.data[1],
        &fd->filt_pos_lla_deg.lla.data[2],
        &fd->filt_pos_lla_deg.timestamp,
        &fd->rpy_rates.data[0],
        &fd->rpy_rates.data[1],
        &fd->rpy_rates.data[2],
        &fd->rpy.data[0],
        &fd->rpy.data[1],
        &fd->rpy.data[2]
    };
    
    for (size_t i = 0; i < sizeof(dbl_fields) / sizeof(dbl_fields[0]); i++) {
        if (offset + 8 > buffer_size) {
            return 0;
        }
        memcpy(buf + offset, dbl_fields[i], 8);
        offset += 8;
    }
    
    // Write uint32_t fields
    memcpy(buf + offset, &fd->custom_mode_id, 4);
    offset += 4;
    uint32_t mode_val = (uint32_t)fd->mode;
    memcpy(buf + offset, &mode_val, 4);
    offset += 4;
    
    // Write bools as uint8_t (packed)
    uint8_t bools = 0;
    bools |= (fd->is_armed ? 1 : 0);
    bools |= (fd->offboardMode ? 2 : 0);
    bools |= (fd->is_available ? 4 : 0);
    bools |= (fd->was_available_once ? 8 : 0);
    bools |= (fd->is_gyrometer_calibration_ok ? 16 : 0);
    bools |= (fd->is_accelerometer_calibration_ok ? 32 : 0);
    bools |= (fd->is_magnetometer_calibration_ok ? 64 : 0);
    bools |= (fd->in_air ? 128 : 0);
    buf[offset++] = bools;
    
    // Write gathered flags as uint32_t bitfield
    uint32_t gathered_flags = 0;
    if (fd->gathered.euler_ned_bodyfrd) gathered_flags |= 1;
    if (fd->gathered.quat_ned_bodyfrd) gathered_flags |= 2;
    if (fd->gathered.pos_ned_m) gathered_flags |= 4;
    if (fd->gathered.vel_ned_m) gathered_flags |= 8;
    if (fd->gathered.imu_ned) gathered_flags |= 16;
    if (fd->gathered.tracker_px) gathered_flags |= 32;
    if (fd->gathered.rpy) gathered_flags |= 64;
    if (fd->gathered.rpy_rates) gathered_flags |= 128;
    if (fd->gathered.custom_mode_id) gathered_flags |= 256;
    if (fd->gathered.mode) gathered_flags |= 512;
    if (fd->gathered.relative_m) gathered_flags |= 1024;
    if (fd->gathered.amsl_m) gathered_flags |= 2048;
    if (fd->gathered.local_m) gathered_flags |= 4096;
    if (fd->gathered.monotonic_m) gathered_flags |= 8192;
    if (fd->gathered.terrain_m) gathered_flags |= 16384;
    if (fd->gathered.bottom_clearance_m) gathered_flags |= 32768;
    if (fd->gathered.absolute_press_hpa) gathered_flags |= 65536;
    if (fd->gathered.differential_press_hpa) gathered_flags |= 131072;
    if (fd->gathered.pressure) gathered_flags |= 262144;
    if (fd->gathered.temperature) gathered_flags |= 524288;
    memcpy(buf + offset, &gathered_flags, 4);
    offset += 4;
    
    // Note: String fields (custom_mode_name, status_text, flight_mode, landing_state) 
    // are not serialized as they are pointers and may not be valid.
    // Python will handle these as None/optional fields.
    
    return offset;
}

static int flight_data_deserialize(flight_data_t* fd, const void* buffer, size_t buffer_size) {
    if (buffer_size < sizeof(flight_data_t)) {
        return -1;
    }
    memcpy(fd, buffer, sizeof(flight_data_t));
    return 0;
}

// Deserialize attitude command from binary format
// Format: magic (4 bytes), version (4 bytes), thrust (8 bytes), rpy_rate[3] (24 bytes), 
//         quat[4] (32 bytes), is_rate (1 byte), padding (3 bytes) = 76 bytes total
static int deserialize_attitude_cmd(const void* data, size_t data_len, quaternion_t* quat, vec3_t* rpy_rate, 
                                     double* thrust, bool* is_rate) {
    if (data == NULL || quat == NULL || rpy_rate == NULL || thrust == NULL || is_rate == NULL) {
        return -1;
    }
    
    // Minimum size check
    if (data_len < 76) {
        return -1;
    }
    
    const uint8_t* buf = (const uint8_t*)data;
    size_t offset = 0;
    
    // Check magic number (0x41545449 = "ATTI" in ASCII)
    uint32_t magic = 0;
    memcpy(&magic, buf + offset, 4);
    offset += 4;
    if (magic != 0x41545449) {
        // Try to deserialize as pickle format (backward compatibility)
        // For now, return error - pickle deserialization would require Python C API
        return -1;
    }
    
    // Read version
    uint32_t version = 0;
    memcpy(&version, buf + offset, 4);
    offset += 4;
    if (version != 1) {
        return -1;
    }
    
    // Read thrust (double)
    memcpy(thrust, buf + offset, 8);
    offset += 8;
    
    // Read rpy_rate (3 doubles)
    memcpy(&rpy_rate->data[0], buf + offset, 8);
    offset += 8;
    memcpy(&rpy_rate->data[1], buf + offset, 8);
    offset += 8;
    memcpy(&rpy_rate->data[2], buf + offset, 8);
    offset += 8;
    
    // Read quaternion (w, x, y, z as 4 doubles)
    double quat_array[4];
    memcpy(quat_array, buf + offset, 32);
    offset += 32;
    quaternion_init(quat, quat_array[1], quat_array[2], quat_array[3], quat_array[0]);  // x, y, z, w
    
    // Read is_rate (uint8_t)
    uint8_t is_rate_val = buf[offset];
    *is_rate = (is_rate_val != 0);
    
    return 0;
}

// Deserialize velocity command from binary format
// Format: magic (4 bytes), version (4 bytes), vel[3] (24 bytes), yaw (8 bytes), 
//         yaw_rate (8 bytes), yaw_valid (1 byte), yaw_rate_valid (1 byte), padding (2 bytes) = 52 bytes total
static int deserialize_vel_cmd(const void* data, size_t data_len, vec3_t* vel, double* yaw, double* yaw_rate) {
    if (data == NULL || vel == NULL || yaw == NULL || yaw_rate == NULL) {
        return -1;
    }
    
    // Minimum size check
    if (data_len < 52) {
        return -1;
    }
    
    const uint8_t* buf = (const uint8_t*)data;
    size_t offset = 0;
    
    // Check magic number (0x56454C43 = "VELC" in ASCII)
    uint32_t magic = 0;
    memcpy(&magic, buf + offset, 4);
    offset += 4;
    if (magic != 0x56454C43) {
        // Try to deserialize as pickle format (backward compatibility)
        // For now, return error - pickle deserialization would require Python C API
        return -1;
    }
    
    // Read version
    uint32_t version = 0;
    memcpy(&version, buf + offset, 4);
    offset += 4;
    if (version != 1) {
        return -1;
    }
    
    // Read velocity (3 doubles)
    memcpy(&vel->data[0], buf + offset, 8);
    offset += 8;
    memcpy(&vel->data[1], buf + offset, 8);
    offset += 8;
    memcpy(&vel->data[2], buf + offset, 8);
    offset += 8;
    
    // Read yaw (double)
    memcpy(yaw, buf + offset, 8);
    offset += 8;
    
    // Read yaw_rate (double)
    memcpy(yaw_rate, buf + offset, 8);
    offset += 8;
    
    // Read validity flags
    uint8_t yaw_valid = buf[offset];
    offset += 1;
    uint8_t yaw_rate_valid = buf[offset];
    offset += 1;
    
    // Set NaN if not valid
    if (!yaw_valid) {
        *yaw = NAN;
    }
    if (!yaw_rate_valid) {
        *yaw_rate = NAN;
    }
    
    return 0;
}

