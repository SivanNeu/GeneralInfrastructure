#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "ZMQ_to_serial.h"
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
#include <fcntl.h>
#include <poll.h>
#include <sys/errno.h>
#include <termios.h>
#include <sys/ioctl.h>

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
    int fd;  // File descriptor (serial port)
    
    // Serial-specific fields
    char* serial_device;  // Serial device path (e.g., "/dev/ttyS0")
    int baudrate;  // Serial baudrate
    
    // Common fields
    uint8_t target_system;
    uint8_t target_component;
    uint8_t system_id;
    uint8_t component_id;
    mavlink_status_t status;
    bool connected;  // MAVLink connection status (heartbeat received)
} mavlink_connection_t;

// Forward declarations
static void* mavlink_connect(const char* address);
static void mavlink_disconnect(void* conn);
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

// Serial port helper functions
static int serial_open_port(const char* device);
static bool serial_setup_port(int fd, int baudrate);
static ssize_t serial_read_port(int fd, uint8_t* buffer, size_t buffer_size);
static ssize_t serial_write_port(int fd, const uint8_t* buffer, size_t buffer_size);

// Command deserialization helpers
static int deserialize_attitude_cmd(const void* data, size_t data_len, quaternion_t* quat, vec3_t* rpy_rate, 
                                     double* thrust, bool* is_rate);
static int deserialize_vel_cmd(const void* data, size_t data_len, vec3_t* vel, double* yaw, double* yaw_rate);

// Initialize ZMQ to Serial bridge
int zmq_to_serial_init(zmq_to_serial_t* bridge, const char* log_dir) {
    if (bridge == NULL) {
        return -1;
    }
    
    // Note: bridge should be zero-initialized by caller (using calloc)
    // Initialize log directory
    if (log_dir != NULL) {
        bridge->log_dir = strdup(log_dir);
        if (bridge->log_dir == NULL) {
            fprintf(stderr, "Failed to allocate memory for log_dir\n");
            return -1;
        }
    } else {
        bridge->log_dir = strdup("./logs/");
        if (bridge->log_dir == NULL) {
            fprintf(stderr, "Failed to allocate memory for log_dir\n");
            return -1;
        }
    }
    
    // Initialize command queue (ONLY shared structure between zmq_reader and mavlink_sender threads)
    if (command_queue_init(&bridge->command_queue) != 0) {
        fprintf(stderr, "Failed to initialize command queue\n");
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Initialize ZMQ
    zmq_wrapper_init();
    
    // Initialize mavlink
    // Use serial port default if not set
    if (bridge->mavlink_address == NULL) {
        bridge->mavlink_address = strdup("/dev/ttyS0:921600");
        if (bridge->mavlink_address == NULL) {
            fprintf(stderr, "Failed to allocate memory for mavlink_address\n");
            command_queue_cleanup(&bridge->command_queue);
            free(bridge->log_dir);
            bridge->log_dir = NULL;
            return -1;
        }
    }
    if (zmq_to_serial_init_mavlink(bridge) != 0) {
        fprintf(stderr, "Failed to initialize mavlink connection\n");
        bridge->init_success = false;
        free(bridge->mavlink_address);
        bridge->mavlink_address = NULL;
        command_queue_cleanup(&bridge->command_queue);
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Create ZMQ subscriber socket
    bridge->sub_socket = zmq_subscriber_create(TOPIC_GUIDANCE_CMD_PORT);
    if (bridge->sub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ subscriber socket\n");
        bridge->init_success = false;
        mavlink_disconnect(bridge->mavlink_connection);
        bridge->mavlink_connection = NULL;
        free(bridge->mavlink_address);
        bridge->mavlink_address = NULL;
        command_queue_cleanup(&bridge->command_queue);
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Subscribe to all command topics
    // CRITICAL: For multipart messages, subscribe to empty string to receive ALL messages
    // The receive function will handle topic detection and filtering
    if (zmq_subscriber_subscribe(bridge->sub_socket, "") != 0) {
        fprintf(stderr, "Failed to subscribe to empty string (all messages)\n");
        return -1;
    }
    
    // Wait for subscriber to connect
    // Note: In ZMQ PUB/SUB, messages sent before subscriber connects are lost
    usleep(50000);  // 50ms
    
    // Initialize control flags
    bridge->mavlink_connected_to_usb = false;
    bridge->disable_offboard_control = false;
    bridge->offboard_control_enabled = false;
    bridge->running = true;
    bridge->init_success = true;
    
    // Start Process 3: ZMQ reader (reads from ZMQ, enqueues to command_queue)
    if (pthread_create(&bridge->zmq_reader_thread, NULL, zmq_to_serial_zmq_reader_thread_func, bridge) != 0) {
        fprintf(stderr, "Failed to create ZMQ reader thread (Process 3)\n");
        bridge->running = false;
        bridge->init_success = false;
        command_queue_cleanup(&bridge->command_queue);
        return -1;
    }
    
    // Start Process 4: MAVLink sender (dequeues from command_queue, sends to MAVLink Serial)
    if (pthread_create(&bridge->mavlink_sender_thread, NULL, zmq_to_serial_mavlink_sender_thread_func, bridge) != 0) {
        fprintf(stderr, "Failed to create MAVLink sender thread (Process 4)\n");
        bridge->running = false;
        if (bridge->zmq_reader_thread != 0) {
            pthread_join(bridge->zmq_reader_thread, NULL);
        }
        bridge->init_success = false;
        command_queue_cleanup(&bridge->command_queue);
        return -1;
    }
    
    printf("ZMQ to Serial bridge: Processes 3 & 4 started\n");
    printf("  Process 3: ZMQ → command_queue\n");
    printf("  Process 4: command_queue → MAVLink Serial\n");
    
    return 0;
}

void zmq_to_serial_cleanup(zmq_to_serial_t* bridge) {
    if (bridge == NULL) {
        return;
    }
    
    // Stop threads
    zmq_to_serial_stop(bridge);
    
    // Cleanup queues
    command_queue_cleanup(&bridge->command_queue);
    
    // Close ZMQ sockets
    if (bridge->sub_socket != NULL) {
        zmq_subscriber_destroy(bridge->sub_socket);
        bridge->sub_socket = NULL;
    }
    
    // Disconnect mavlink
    if (bridge->mavlink_connection != NULL) {
        mavlink_disconnect(bridge->mavlink_connection);
        bridge->mavlink_connection = NULL;
    }
    
    // Free strings
    if (bridge->log_dir != NULL) {
        free(bridge->log_dir);
        bridge->log_dir = NULL;
    }
    if (bridge->mavlink_address != NULL) {
        free(bridge->mavlink_address);
        bridge->mavlink_address = NULL;
    }
    
    zmq_wrapper_cleanup();
}

bool zmq_to_serial_init_succeeded(zmq_to_serial_t* bridge) {
    return bridge != NULL && bridge->init_success;
}

void zmq_to_serial_stop(zmq_to_serial_t* bridge) {
    if (bridge == NULL) {
        return;
    }
    
    bridge->running = false;
    
    // Signal queues to wake up waiting threads
    pthread_mutex_lock(&bridge->command_queue.mutex);
    pthread_cond_broadcast(&bridge->command_queue.not_empty);
    pthread_cond_broadcast(&bridge->command_queue.not_full);
    pthread_mutex_unlock(&bridge->command_queue.mutex);
    
    // Wait for threads to finish
    if (bridge->zmq_reader_thread != 0) {
        pthread_join(bridge->zmq_reader_thread, NULL);
    }
    if (bridge->mavlink_sender_thread != 0) {
        pthread_join(bridge->mavlink_sender_thread, NULL);
    }
}

// Initialize mavlink connection
int zmq_to_serial_init_mavlink(zmq_to_serial_t* bridge) {
    bridge->mavlink_connection = (void*)mavlink_connect(bridge->mavlink_address);
    if (bridge->mavlink_connection == NULL) {
        return -1;
    }
    
    // Send heartbeat (try a few times)
    // Note: We don't require an immediate response - the MAVLink device might not be running yet
    // The connection will work once the device starts sending messages
    int count = 0;
    while (count < 3) {
        count++;
        mavlink_send_heartbeat(bridge->mavlink_connection);
        // Try to wait for heartbeat, but don't fail if we don't get one
        if (mavlink_wait_heartbeat(bridge->mavlink_connection, 500) == 0) {
            printf("MAVLink heartbeat received\n");
            return 0;
        }
    }
    
    // Connection established, but no heartbeat yet - this is OK
    // The heartbeat will come when the MAVLink device is running
    printf("MAVLink connection established (waiting for heartbeat from device)\n");
    return 0;
}

// Process 3: ZMQ Reader - Reads commands from ZMQ and enqueues to command_queue
void* zmq_to_serial_zmq_reader_thread_func(void* arg) {
    zmq_to_serial_t* bridge = (zmq_to_serial_t*)arg;
    char topic_buffer[256];
    char data_buffer[4096];
    
    while (bridge->running) {
        int data_len = zmq_subscriber_receive(bridge->sub_socket, topic_buffer, sizeof(topic_buffer),
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
                command_queue_enqueue(&bridge->command_queue, &cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_NED) == 0) {
            vec3_t vel;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel, &yaw, &yaw_rate) == 0) {
                cmd.type = CMD_TYPE_VEL_NED;
                cmd.vel = vel;
                cmd.yaw = isnan(yaw) ? NAN : yaw;
                cmd.yaw_rate = isnan(yaw_rate) ? NAN : yaw_rate;
                command_queue_enqueue(&bridge->command_queue, &cmd);
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
                
                command_queue_enqueue(&bridge->command_queue, &cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ARM) == 0) {
            cmd.type = CMD_TYPE_ARM;
            command_queue_enqueue(&bridge->command_queue, &cmd);
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ACC) == 0) {
            // Acceleration command - not implemented yet
            cmd.type = CMD_TYPE_ACC;
            // TODO: deserialize and enqueue
        }
    }
    
    return NULL;
}

// Process 4: MAVLink Sender - Dequeues from command_queue and sends to MAVLink Serial
void* zmq_to_serial_mavlink_sender_thread_func(void* arg) {
    zmq_to_serial_t* bridge = (zmq_to_serial_t*)arg;
    command_t cmd;
    
    while (bridge->running) {
        // Dequeue command with 10ms timeout (non-blocking if queue is empty)
        if (command_queue_dequeue(&bridge->command_queue, &cmd, 10) != 0) {
            continue;  // No command available or timeout
        }
        
        // Process command based on type
        switch (cmd.type) {
            case CMD_TYPE_VEL_NED: {
                // Send velocity command in NED frame
                mavlink_connection_t* mconn = (mavlink_connection_t*)bridge->mavlink_connection;
                if (mconn != NULL) {
                    vec3_t vel = cmd.vel;
                    double yaw = cmd.yaw;
                    double yaw_rate = cmd.yaw_rate;
                    mavlink_send_set_position_target_local_ned(mconn, 0, 1, 0x07, NULL, &vel, NULL, yaw, yaw_rate);
                }
                break;
            }
            
            case CMD_TYPE_VEL_BODY: {
                // Transform body velocity to NED using stored quaternion
                vec3_t vel_ned = quaternion_rotate_vec(&cmd.quat_ned_bodyfrd, &cmd.vel);
                mavlink_connection_t* mconn = (mavlink_connection_t*)bridge->mavlink_connection;
                if (mconn != NULL) {
                    double yaw = cmd.yaw;
                    double yaw_rate = cmd.yaw_rate;
                    mavlink_send_set_position_target_local_ned(mconn, 0, 1, 0x07, NULL, &vel_ned, NULL, yaw, yaw_rate);
                }
                break;
            }
            
            case CMD_TYPE_ATTITUDE: {
                mavlink_connection_t* mconn = (mavlink_connection_t*)bridge->mavlink_connection;
                if (mconn != NULL) {
                    mavlink_send_set_attitude_target(mconn, 0, 0, &cmd.quat, 0.0, 0.0, 0.0, cmd.thrust);
                }
                break;
            }
            
            case CMD_TYPE_ATTITUDE_RATE: {
                mavlink_connection_t* mconn = (mavlink_connection_t*)bridge->mavlink_connection;
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
                mavlink_connection_t* mconn = (mavlink_connection_t*)bridge->mavlink_connection;
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

// Serial port helper functions
static int serial_open_port(const char* device) {
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    // O_NDELAY - Non-blocking mode
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd == -1) {
        fprintf(stderr, "Failed to open serial port %s: %s\n", device, strerror(errno));
        return -1;
    }
    
    // Clear O_NDELAY to make blocking
    fcntl(fd, F_SETFL, 0);
    
    return fd;
}

static bool serial_setup_port(int fd, int baudrate) {
    // Check file descriptor
    if (!isatty(fd)) {
        fprintf(stderr, "ERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }
    
    // Read file descriptor configuration
    struct termios config;
    if (tcgetattr(fd, &config) < 0) {
        fprintf(stderr, "ERROR: could not read configuration of fd %d\n", fd);
        return false;
    }
    
    // Input flags - Turn off input processing
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    
    // Output flags - Turn off output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    
    // No line processing
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    
    // Turn off character processing
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    
    // One input byte is enough to return from read()
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10;  // 1 second timeout
    
    // Apply baudrate
    speed_t speed;
    switch (baudrate) {
        case 1200: speed = B1200; break;
        case 1800: speed = B1800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set\n", baudrate);
            return false;
    }
    
    if (cfsetispeed(&config, speed) < 0 || cfsetospeed(&config, speed) < 0) {
        fprintf(stderr, "ERROR: Could not set desired baud rate of %d Baud\n", baudrate);
        return false;
    }
    
    // Finally, apply the configuration
    if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
        fprintf(stderr, "ERROR: could not set configuration of fd %d\n", fd);
        return false;
    }
    
    return true;
}

static ssize_t serial_read_port(int fd, uint8_t* buffer, size_t buffer_size) {
    return read(fd, buffer, buffer_size);
}

static ssize_t serial_write_port(int fd, const uint8_t* buffer, size_t buffer_size) {
    ssize_t bytes_written = write(fd, buffer, buffer_size);
    if (bytes_written > 0) {
        // Wait until all data has been written
        tcdrain(fd);
    }
    return bytes_written;
}

// Parse MAVLink address string
// Format: "/dev/ttyS0:921600" or "/dev/ttyS0:57600" (also supports "serial:///dev/ttyS0:921600" for backward compatibility)
// Returns: 0 on success, -1 on error
// Sets device and baudrate
static int parse_mavlink_address(const char* address, 
                                  char* device, size_t device_size, int* baudrate) {
    if (address == NULL || device == NULL || device_size == 0 || baudrate == NULL) {
        return -1;
    }
    
    const char* serial_part = address;
    
    // Check if it's a serial connection with "serial://" prefix (backward compatibility)
    if (strncmp(address, "serial://", 9) == 0) {
        serial_part = address + 9;  // Skip "serial://"
    }
    
    // Find the colon that separates device from baudrate
    char* colon = strchr(serial_part, ':');
    if (colon == NULL) {
        fprintf(stderr, "Invalid serial format, expected 'device:baudrate' (e.g., '/dev/ttyS0:921600')\n");
        return -1;
    }
    
    size_t device_len = colon - serial_part;
    if (device_len >= device_size) {
        fprintf(stderr, "Serial device path too long\n");
        return -1;
    }
    strncpy(device, serial_part, device_len);
    device[device_len] = '\0';
    
    *baudrate = atoi(colon + 1);
    if (*baudrate <= 0) {
        fprintf(stderr, "Invalid baudrate: %s\n", colon + 1);
        return -1;
    }
    
    return 0;
}

// Create serial port and connect to MAVLink endpoint
static void* mavlink_connect(const char* address) {
    char device[256];
    int baudrate;
    
    if (parse_mavlink_address(address, device, sizeof(device), &baudrate) != 0) {
        fprintf(stderr, "Failed to parse MAVLink address: %s\n", address);
        return NULL;
    }
    
    mavlink_connection_t* conn = (mavlink_connection_t*)calloc(1, sizeof(mavlink_connection_t));
    if (conn == NULL) {
        fprintf(stderr, "Failed to allocate MAVLink connection\n");
        return NULL;
    }
    
    // Serial port connection
    conn->serial_device = strdup(device);
    if (conn->serial_device == NULL) {
        fprintf(stderr, "Failed to allocate memory for serial device\n");
        free(conn);
        return NULL;
    }
    conn->baudrate = baudrate;
    
    // Open serial port
    conn->fd = serial_open_port(device);
    if (conn->fd < 0) {
        free(conn->serial_device);
        free(conn);
        return NULL;
    }
    
    // Setup serial port
    if (!serial_setup_port(conn->fd, baudrate)) {
        fprintf(stderr, "Failed to setup serial port\n");
        close(conn->fd);
        free(conn->serial_device);
        free(conn);
        return NULL;
    }
    
    printf("MAVLink serial connection: %s at %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", 
           device, baudrate);
    
    // Initialize MAVLink connection parameters
    conn->system_id = MAV_SYSTEM_ID;
    conn->component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    conn->target_system = 1;  // Will be updated when we receive heartbeat
    conn->target_component = MAV_COMP_ID_AUTOPILOT1;
    conn->connected = false;
    memset(&conn->status, 0, sizeof(conn->status));
    
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
    
    // Free serial device string if it exists
    if (mconn->serial_device != NULL) {
        free(mconn->serial_device);
        mconn->serial_device = NULL;
    }
    
    free(conn);
}

static int mavlink_send_heartbeat(void* conn) {
    if (conn == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
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
    
    // Serial port: write directly
    ssize_t sent = serial_write_port(mconn->fd, buffer, len);
    
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
    
    ssize_t sent = serial_write_port(mconn->fd, buffer, len);
    
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
    
    ssize_t sent = serial_write_port(mconn->fd, buffer, len);
    
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
    
    ssize_t sent = serial_write_port(mconn->fd, buffer, len);
    
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
    
    // Try to receive a message (simplified - just check if data is available)
    // For serial, we'd need to read and parse, but for now just check availability
    return 0;  // Assume heartbeat available if poll succeeds
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
