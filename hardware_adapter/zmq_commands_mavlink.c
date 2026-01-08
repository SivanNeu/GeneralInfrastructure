#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "zmq_commands_mavlink.h"
#include "command_queue.h"
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
    struct sockaddr_in remote_addr;  // Address we receive from (port 14540, set from received messages)
    struct sockaddr_in send_addr;    // Address we send to (port 14030, same IP as remote_addr)
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
    
    // Initialize command queue (ONLY shared structure between zmq_reader and mavlink_sender threads)
    // Only initialize if command processing pair is enabled
#if ENABLE_COMMAND_PROCESSING_PAIR
    if (command_queue_init(&adapter->command_queue) != 0) {
        fprintf(stderr, "Failed to initialize command queue\n");
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
#endif
    
    // Initialize ZMQ
    zmq_wrapper_init();
    
    // Initialize mavlink
    // Use server mode (udp:MAVLINK_DEFAULT_UDP_PORT) to listen on default port, matching the working UDP example
    // This allows receiving messages from the MAVLink server which sends to the default port
    char default_address[64];
    snprintf(default_address, sizeof(default_address), "udp:%d", MAVLINK_DEFAULT_UDP_PORT);
    adapter->mavlink_address = strdup(default_address);
    if (adapter->mavlink_address == NULL) {
        fprintf(stderr, "Failed to allocate memory for mavlink_address\n");
#if ENABLE_COMMAND_PROCESSING_PAIR
        command_queue_cleanup(&adapter->command_queue);
#endif
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    if (hardware_adapter_init_mavlink(adapter) != 0) {
        fprintf(stderr, "Failed to initialize mavlink connection\n");
        adapter->init_success = false;
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
#if ENABLE_COMMAND_PROCESSING_PAIR
        command_queue_cleanup(&adapter->command_queue);
#endif
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Create ZMQ subscriber socket (only needed if command processing pair is enabled)
#if ENABLE_COMMAND_PROCESSING_PAIR
    adapter->sub_socket = zmq_subscriber_create(TOPIC_GUIDANCE_CMD_PORT);
    if (adapter->sub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ subscriber socket\n");
        adapter->init_success = false;
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
#if ENABLE_COMMAND_PROCESSING_PAIR
        command_queue_cleanup(&adapter->command_queue);
#endif
        free(adapter->log_dir);
        adapter->log_dir = NULL;
        return -1;
    }
    
    // Subscribe to all command topics
    // CRITICAL: For multipart messages, subscribe to empty string to receive ALL messages
    // The receive function will handle topic detection and filtering
    // This avoids issues with ZMQ subscription filters potentially stripping topic frames
    if (zmq_subscriber_subscribe(adapter->sub_socket, "") != 0) {
        fprintf(stderr, "Failed to subscribe to empty string (all messages)\n");
        return -1;
    }
    
    // Wait for subscriber to connect
    // Note: In ZMQ PUB/SUB, messages sent before subscriber connects are lost
    usleep(50000);  // 50ms (reduced from 500ms)
#else
    adapter->sub_socket = NULL;  // Not needed if command processing pair is disabled
#endif
    
    // Initialize control flags
    adapter->mavlink_connected_to_usb = false;
    adapter->disable_offboard_control = false;
    adapter->offboard_control_enabled = false;
    adapter->running = true;
    adapter->init_success = true;
    
    // Start Process 3: ZMQ reader (reads from ZMQ, enqueues to command_queue)
    // Only start if command processing pair is enabled at compile time
#if ENABLE_COMMAND_PROCESSING_PAIR
    {
        if (pthread_create(&adapter->zmq_reader_thread, NULL, hardware_adapter_zmq_reader_thread_func, adapter) != 0) {
            fprintf(stderr, "Failed to create ZMQ reader thread (Process 3)\n");
            adapter->running = false;
#if ENABLE_DATA_PROCESSING_PAIR
            if (adapter->mavlink_reader_thread != 0) {
                pthread_join(adapter->mavlink_reader_thread, NULL);
            }
            if (adapter->data_processor_thread != 0) {
                pthread_join(adapter->data_processor_thread, NULL);
            }
#endif
            adapter->init_success = false;
#if ENABLE_DATA_PROCESSING_PAIR
            mavlink_data_queue_cleanup(&adapter->data_queue);
#endif
#if ENABLE_COMMAND_PROCESSING_PAIR
            command_queue_cleanup(&adapter->command_queue);
#endif
            return -1;
        }
        
        // Start Process 4: MAVLink sender (dequeues from command_queue, sends to MAVLink UDP)
        if (pthread_create(&adapter->mavlink_sender_thread, NULL, hardware_adapter_mavlink_sender_thread_func, adapter) != 0) {
            fprintf(stderr, "Failed to create MAVLink sender thread (Process 4)\n");
            adapter->running = false;
            if (adapter->zmq_reader_thread != 0) {
                pthread_join(adapter->zmq_reader_thread, NULL);
            }
#if ENABLE_DATA_PROCESSING_PAIR
            if (adapter->mavlink_reader_thread != 0) {
                pthread_join(adapter->mavlink_reader_thread, NULL);
            }
            if (adapter->data_processor_thread != 0) {
                pthread_join(adapter->data_processor_thread, NULL);
            }
#endif
            adapter->init_success = false;
#if ENABLE_DATA_PROCESSING_PAIR
            mavlink_data_queue_cleanup(&adapter->data_queue);
#endif
#if ENABLE_COMMAND_PROCESSING_PAIR
            command_queue_cleanup(&adapter->command_queue);
#endif
            return -1;
        }
    }
#endif
    
    // Print status based on compile-time defines only
#if ENABLE_COMMAND_PROCESSING_PAIR
    printf("Hardware adapter: Processes 3 & 4 started (command processing pair enabled at compile time)\n");
    printf("  Process 3: ZMQ → command_queue\n");
    printf("  Process 4: command_queue → MAVLink UDP\n");
#else
    printf("Hardware adapter: Command processing pair disabled at compile time\n");
#endif
    
    return 0;
}

void hardware_adapter_cleanup(hardware_adapter_t* adapter) {
    if (adapter == NULL) {
        return;
    }
    
    // Stop threads
    hardware_adapter_stop(adapter);
    
    // Cleanup queues (ONLY shared structures)
    // Only cleanup queues that were initialized (based on compile-time defines)
#if ENABLE_COMMAND_PROCESSING_PAIR
    command_queue_cleanup(&adapter->command_queue);  // Shared between processes 3 and 4
#endif
    
    // Close ZMQ sockets
    if (adapter->sub_socket != NULL) {
        zmq_subscriber_destroy(adapter->sub_socket);
        adapter->sub_socket = NULL;
    }
    
    // Disconnect mavlink
    if (adapter->mavlink_connection != NULL) {
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
    }
    
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
    
    // Signal queues to wake up waiting threads (only if queues were initialized)
#if ENABLE_COMMAND_PROCESSING_PAIR
    pthread_mutex_lock(&adapter->command_queue.mutex);
    pthread_cond_broadcast(&adapter->command_queue.not_empty);
    pthread_cond_broadcast(&adapter->command_queue.not_full);
    pthread_mutex_unlock(&adapter->command_queue.mutex);
#endif
    
    // Wait for threads to finish (only if they were created)
#if ENABLE_COMMAND_PROCESSING_PAIR
    if (adapter->zmq_reader_thread != 0) {
        pthread_join(adapter->zmq_reader_thread, NULL);
    }
    if (adapter->mavlink_sender_thread != 0) {
        pthread_join(adapter->mavlink_sender_thread, NULL);
    }
#endif
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
        if (mavlink_wait_heartbeat(adapter->mavlink_connection, 500) == 0) {
            printf("MAVLink heartbeat received\n");
            return 0;
        }
    }
    
    // Connection established, but no heartbeat yet - this is OK
    // The heartbeat will come when the MAVLink server is running
    printf("MAVLink connection established (waiting for heartbeat from server)\n");
    return 0;
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

// Process 3: ZMQ Reader - Reads commands from ZMQ and enqueues to command_queue (ONLY shared structure with process 4)
void* hardware_adapter_zmq_reader_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    char topic_buffer[256];
    char data_buffer[4096];
    
    while (adapter->running) {
        int data_len = zmq_subscriber_receive(adapter->sub_socket, topic_buffer, sizeof(topic_buffer),
                                             data_buffer, sizeof(data_buffer), 0);  // 0ms timeout - non-blocking
        
        if (data_len == -2) {
            // Message received but topic didn't match
            continue;
        }
        
        if (data_len < 0) {
            continue;  // No message available
        }
        
        // Parse command and enqueue it
        command_t cmd = {0};
        
        if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ATTITUDE) == 0) {
            quaternion_t quat;
            vec3_t rpy_rate;
            double thrust;
            bool is_rate;
            
            if (deserialize_attitude_cmd(data_buffer, data_len, &quat, &rpy_rate, &thrust, &is_rate) == 0) {
                if (is_rate) {
                    cmd.type = CMD_TYPE_ATTITUDE_RATE;
                    cmd.rpy_rate = rpy_rate;
                    cmd.thrust = thrust;
                } else {
                    cmd.type = CMD_TYPE_ATTITUDE;
                    cmd.quat = quat;
                    cmd.thrust = thrust;
                }
                cmd.is_rate = is_rate;
                command_queue_enqueue(&adapter->command_queue, &cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_NED) == 0) {
            vec3_t vel;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel, &yaw, &yaw_rate) == 0) {
                cmd.type = CMD_TYPE_VEL_NED;
                cmd.vel = vel;
                cmd.yaw = isnan(yaw) ? NAN : yaw;
                cmd.yaw_rate = isnan(yaw_rate) ? NAN : yaw_rate;
                command_queue_enqueue(&adapter->command_queue, &cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_BODY) == 0) {
            vec3_t vel_body;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel_body, &yaw, &yaw_rate) == 0) {
                cmd.type = CMD_TYPE_VEL_BODY;
                cmd.vel = vel_body;  // Store body velocity
                cmd.yaw = isnan(yaw) ? NAN : yaw;
                cmd.yaw_rate = isnan(yaw_rate) ? NAN : yaw_rate;
                
                // For VEL_BODY: Cannot transform without quaternion (no shared state allowed)
                // Store body velocity as-is - process 4 will need to handle transformation
                // For now, use identity quaternion (no transformation)
                quaternion_init(&cmd.quat_ned_bodyfrd, 0.0, 0.0, 0.0, 1.0);
                
                command_queue_enqueue(&adapter->command_queue, &cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ARM) == 0) {
            cmd.type = CMD_TYPE_ARM;
            command_queue_enqueue(&adapter->command_queue, &cmd);
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ACC) == 0) {
            // Acceleration command - not implemented yet
            cmd.type = CMD_TYPE_ACC;
            // TODO: deserialize and enqueue
        }
    }
    
    return NULL;
}

// Process 4: MAVLink Sender - Dequeues from command_queue and sends to MAVLink UDP (ONLY shared structure with process 3)
void* hardware_adapter_mavlink_sender_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    command_t cmd;
    
    while (adapter->running) {
        // Dequeue command with 10ms timeout (non-blocking if queue is empty)
        if (command_queue_dequeue(&adapter->command_queue, &cmd, 10) != 0) {
            continue;  // No command available or timeout
        }
        
        // Process command based on type
        switch (cmd.type) {
            case CMD_TYPE_VEL_NED: {
                // Send velocity command in NED frame
                mavlink_connection_t* mconn = (mavlink_connection_t*)adapter->mavlink_connection;
                if (mconn != NULL) {
                    vec3_t vel = cmd.vel;
                    double yaw = cmd.yaw;
                    double yaw_rate = cmd.yaw_rate;
                    int result = mavlink_send_set_position_target_local_ned(mconn, 0, 1, 0x07, NULL, &vel, NULL, yaw, yaw_rate);
                    static int cmd_count = 0;
                    if (++cmd_count % 50 == 0) {  // Print every 50th command to avoid spam
                        char send_ip[INET_ADDRSTRLEN];
                        inet_ntop(AF_INET, &mconn->send_addr.sin_addr, send_ip, INET_ADDRSTRLEN);
                        printf("DEBUG: Sent VEL_NED command #%d, result=%d, target_system=%d, target_component=%d, to %s:%d, vel=(%.2f,%.2f,%.2f), yaw_rate=%.2f\n",
                               cmd_count, result, mconn->target_system, mconn->target_component, 
                               send_ip, ntohs(mconn->send_addr.sin_port),
                               vel.data[0], vel.data[1], vel.data[2], yaw_rate);
                    }
                }
                break;
            }
            
            case CMD_TYPE_VEL_BODY: {
                // Transform body velocity to NED using stored quaternion
                vec3_t vel_ned = quaternion_rotate_vec(&cmd.quat_ned_bodyfrd, &cmd.vel);
                mavlink_connection_t* mconn = (mavlink_connection_t*)adapter->mavlink_connection;
                if (mconn != NULL) {
                    double yaw = cmd.yaw;
                    double yaw_rate = cmd.yaw_rate;
                    mavlink_send_set_position_target_local_ned(mconn, 0, 1, 0x07, NULL, &vel_ned, NULL, yaw, yaw_rate);
                }
                break;
            }
            
            case CMD_TYPE_ATTITUDE: {
                mavlink_connection_t* mconn = (mavlink_connection_t*)adapter->mavlink_connection;
                if (mconn != NULL) {
                    mavlink_send_set_attitude_target(mconn, 0, 0, &cmd.quat, 0.0, 0.0, 0.0, cmd.thrust);
                }
                break;
            }
            
            case CMD_TYPE_ATTITUDE_RATE: {
                mavlink_connection_t* mconn = (mavlink_connection_t*)adapter->mavlink_connection;
                if (mconn != NULL) {
                    mavlink_send_set_attitude_target(mconn, 0, 0x01, NULL, 
                                                     cmd.rpy_rate.data[0], 
                                                     cmd.rpy_rate.data[1], 
                                                     cmd.rpy_rate.data[2], 
                                                     cmd.thrust);
                }
                break;
            }
            
            case CMD_TYPE_ARM: {
                mavlink_connection_t* mconn = (mavlink_connection_t*)adapter->mavlink_connection;
                if (mconn != NULL) {
                    mavlink_send_command_long(mconn, 400, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                }
                break;
            }
            
            case CMD_TYPE_ACC:
                // Not implemented
                break;
        }
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
    *port = MAVLINK_DEFAULT_UDP_PORT;
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
    tv.tv_usec = 10000;  // 10ms timeout (reduced from 100ms)
    if (setsockopt(conn->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        // Removed warning print to reduce overhead
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
    
    // Set up remote address (where we receive messages from - port 14540)
    // For server mode, remote address will be set from first received message
    // For client mode, set it to the specified IP:PORT (receiving port)
    memset(&conn->remote_addr, 0, sizeof(conn->remote_addr));
    if (!is_server_mode) {
        conn->remote_addr.sin_family = AF_INET;
        conn->remote_addr.sin_port = htons(port);  // Receiving port
        if (inet_pton(AF_INET, ip, &conn->remote_addr.sin_addr) != 1) {
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
    
    // Set up send address (where we send commands to)
    // For client mode, use the same IP and port as receiving address
    // For server mode, default to localhost, will be updated from first received message
    // NOTE: We send to the same port the drone sends FROM (not a fixed port like 14030)
    // This matches how pymavlink works: bidirectional connection on the same port
    memset(&conn->send_addr, 0, sizeof(conn->send_addr));
    conn->send_addr.sin_family = AF_INET;
    if (!is_server_mode) {
        // Client mode: use the same IP and port as receiving address
        conn->send_addr.sin_addr = conn->remote_addr.sin_addr;
        conn->send_addr.sin_port = conn->remote_addr.sin_port;
    } else {
        // Server mode: default to localhost, will be updated from first received message
        // This allows sending commands even if data processing pair is disabled
        if (inet_pton(AF_INET, "127.0.0.1", &conn->send_addr.sin_addr) != 1) {
            conn->send_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // Fallback: 127.0.0.1
        }
        // Port will be set from first received message
        conn->send_addr.sin_port = 0;
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
        printf("MAVLink listening on port %d (server mode) for receiving data\n", port);
        printf("MAVLink will send commands to the same port messages are received FROM\n");
    } else {
        printf("MAVLink client mode: receiving from %s:%d, sending commands to same address\n", ip, port);
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
        // Removed error logging to reduce overhead
        return -1;
    }
    
    // Update remote address from received message (for server mode)
    // This allows us to know where messages come from
    // Mutex-free write: this happens once early, then only read by sine test thread
    // We update atomically by copying the whole struct (memcpy is atomic for small structs)
    if (recv_len > 0 && (mconn->remote_addr.sin_addr.s_addr == INADDR_ANY || mconn->remote_addr.sin_port == 0)) {
        // Update remote address (receiving port 14540)
        // Copy atomically - struct sockaddr_in is small enough that memcpy is effectively atomic
        memcpy(&mconn->remote_addr, &src_addr, sizeof(src_addr));
    }
    
    // Always update send address from received message
    // For UDP, we should send to the same port the drone is sending FROM (not a fixed port)
    // This matches how pymavlink works: it sends to the port it receives from
    if (recv_len > 0) {
        char old_ip[INET_ADDRSTRLEN], new_ip[INET_ADDRSTRLEN];
        uint16_t old_port = ntohs(mconn->send_addr.sin_port);
        inet_ntop(AF_INET, &mconn->send_addr.sin_addr, old_ip, INET_ADDRSTRLEN);
        mconn->send_addr.sin_addr = src_addr.sin_addr;  // Update IP from source
        mconn->send_addr.sin_family = AF_INET;
        mconn->send_addr.sin_port = src_addr.sin_port;  // Send to the same port we receive FROM
        inet_ntop(AF_INET, &mconn->send_addr.sin_addr, new_ip, INET_ADDRSTRLEN);
        uint16_t new_port = ntohs(mconn->send_addr.sin_port);
        // Debug: print when send address is updated (only first time to avoid spam)
        static bool send_addr_updated = false;
        if (!send_addr_updated) {
            printf("DEBUG: Updated send address from %s:%d to %s:%d (using source port from received messages)\n", 
                   old_ip, old_port, new_ip, new_port);
            send_addr_updated = true;
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
            // Mutex-free write: uint8_t assignments are naturally atomic
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mconn->target_system = msg.sysid;  // uint8_t write is atomic
                mconn->target_component = msg.compid;  // uint8_t write is atomic
                mconn->connected = true;  // bool write is atomic
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
    
    // Don't send until we've received a message (to know the IP address)
    if (mconn->send_addr.sin_addr.s_addr == INADDR_ANY) {
        static int warn_count = 0;
        if (warn_count++ < 3) {  // Only warn first 3 times to avoid spam
            fprintf(stderr, "WARNING: Cannot send command - send address not set yet (waiting for first received message)\n");
        }
        return -1;  // No send address set yet (waiting for first received message)
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
    
    // Always use sendto for UDP, send to port 14030
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->send_addr, sizeof(mconn->send_addr));
    
    // Removed debug prints to reduce overhead
    
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
    
    // Don't send until we've received a message (to know the IP address)
    if (mconn->send_addr.sin_addr.s_addr == INADDR_ANY) {
        static int warn_count = 0;
        if (warn_count++ < 3) {  // Only warn first 3 times to avoid spam
            fprintf(stderr, "WARNING: Cannot send command - send address not set yet (waiting for first received message)\n");
        }
        return -1;  // No send address set yet (waiting for first received message)
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
    
    // Always use sendto for UDP, send to port 14030
    char send_ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &mconn->send_addr.sin_addr, send_ip_str, INET_ADDRSTRLEN);
    uint16_t send_port = ntohs(mconn->send_addr.sin_port);
    
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->send_addr, sizeof(mconn->send_addr));
    
    if (sent != len) {
        fprintf(stderr, "WARNING: Failed to send command to %s:%d, sent=%zd, expected=%d, errno=%d (%s)\n",
                send_ip_str, send_port, sent, len, errno, strerror(errno));
    }
    
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
    
    // Don't send until we've received a message (to know the IP address)
    if (mconn->send_addr.sin_addr.s_addr == INADDR_ANY) {
        static int warn_count = 0;
        if (warn_count++ < 3) {  // Only warn first 3 times to avoid spam
            fprintf(stderr, "WARNING: Cannot send command - send address not set yet (waiting for first received message)\n");
        }
        return -1;  // No send address set yet (waiting for first received message)
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
    
    // Always use sendto for UDP, send to port 14030
    char send_ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &mconn->send_addr.sin_addr, send_ip_str, INET_ADDRSTRLEN);
    uint16_t send_port = ntohs(mconn->send_addr.sin_port);
    
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->send_addr, sizeof(mconn->send_addr));
    
    if (sent != len) {
        fprintf(stderr, "WARNING: Failed to send command to %s:%d, sent=%zd, expected=%d, errno=%d (%s)\n",
                send_ip_str, send_port, sent, len, errno, strerror(errno));
    }
    
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
    
    // Don't send until we've received a message (to know the IP address)
    if (mconn->send_addr.sin_addr.s_addr == INADDR_ANY) {
        static int warn_count = 0;
        if (warn_count++ < 3) {  // Only warn first 3 times to avoid spam
            fprintf(stderr, "WARNING: Cannot send command - send address not set yet (waiting for first received message)\n");
        }
        return -1;  // No send address set yet (waiting for first received message)
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
    
    // Always use sendto for UDP, send to port 14030
    char send_ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &mconn->send_addr.sin_addr, send_ip_str, INET_ADDRSTRLEN);
    uint16_t send_port = ntohs(mconn->send_addr.sin_port);
    
    ssize_t sent = sendto(mconn->fd, buffer, len, 0,
                         (struct sockaddr*)&mconn->send_addr, sizeof(mconn->send_addr));
    
    if (sent != len) {
        fprintf(stderr, "WARNING: Failed to send command to %s:%d, sent=%zd, expected=%d, errno=%d (%s)\n",
                send_ip_str, send_port, sent, len, errno, strerror(errno));
    }
    
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


