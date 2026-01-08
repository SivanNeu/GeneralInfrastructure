#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <getopt.h>
#include <signal.h>

// MAVLink includes
#include <mavlink/common/mavlink.h>

// ZMQ includes
#include "../hardware_adapter/zmq_wrapper.h"
#include "../hardware_adapter/zmq_topics.h"

// MAVLink constants
#ifndef MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 255
#endif
#ifndef MAV_COMP_ID_ONBOARD_COMPUTER
#define MAV_COMP_ID_ONBOARD_COMPUTER 191
#endif

// PX4 mode constants
#define PX4_FLIGHT_STATE_OFFBOARD 393216
#define PX4_FLIGHT_STATE_HOLD 50593792

// Sine test configuration
#define SINE_VX_AMPL 5.0
#define SINE_VY_AMPL 5.0
#define SINE_VZ_AMPL 1.0
#define SINE_YAWRATE_AMPL 3.0
#define SINE_VX_FREQ 1.0
#define SINE_VY_FREQ 1.0
#define SINE_VZ_FREQ 0.2
#define SINE_YAWRATE_FREQ 1.0
#define SINE_CMD_FREQ 20.0
#define SINE_DURATION 20.0

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
    bool connected;
} mavlink_connection_t;

// Program options
typedef struct {
    char* logfile;
    bool use_sine;
    bool use_zmq;
    bool loop;
    double speed;
    double sine_duration;
    double sine_cmd_freq;
    char* udp_address;
    int udp_port;
    int target_system;
    int target_component;
    int frame;
} options_t;

static mavlink_connection_t* g_mavlink_conn = NULL;
static void* g_zmq_pub = NULL;
static bool g_running = true;

// Forward declarations
static mavlink_connection_t* mavlink_connect(const char* address, int port);
static void mavlink_disconnect(mavlink_connection_t* conn);
static int mavlink_send_heartbeat(mavlink_connection_t* conn);
static int mavlink_send_set_position_target_local_ned(mavlink_connection_t* conn, uint32_t time_boot_ms,
                                                      uint8_t coordinate_frame, uint16_t type_mask,
                                                      float pos_x, float pos_y, float pos_z,
                                                      float vel_x, float vel_y, float vel_z,
                                                      float acc_x, float acc_y, float acc_z,
                                                      float yaw, float yaw_rate);
static int mavlink_send_command_long(mavlink_connection_t* conn, uint16_t command,
                                     float param1, float param2, float param3,
                                     float param4, float param5, float param6, float param7);
static int mavlink_wait_heartbeat(mavlink_connection_t* conn, int timeout_ms);
static int mavlink_receive_heartbeat(mavlink_connection_t* conn, uint32_t* custom_mode);
static void serialize_vel_cmd(double vx, double vy, double vz, double yaw, double yaw_rate, uint8_t* buffer, size_t* len);
static void set_mode_offboard(mavlink_connection_t* conn);
static void set_mode_hold(mavlink_connection_t* conn);
static void replay_log_once(const char* logfile, double speed, bool use_zmq);
static void run_sine_test(double duration, double cmd_freq, bool use_zmq);
static void print_usage(const char* program_name);
static void parse_args(int argc, char* argv[], options_t* opts);
static void signal_handler(int sig);

// Signal handler
void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

// Print usage
void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --log FILE           Path to hardware_adapter CSV log for replay\n");
    printf("  --sine               Send sine-wave velocity commands (default if no --log)\n");
    printf("  --direct-mavlink     Use direct MAVLink UDP instead of ZMQ (default: ZMQ)\n");
    printf("  --loop               Loop the log file continuously\n");
    printf("  --speed FACTOR       Replay speed factor (default: 1.0)\n");
    printf("  --sine-duration SEC  Sine test duration in seconds (default: %.1f)\n", SINE_DURATION);
    printf("  --sine-cmd-freq HZ   Command frequency for sine test (default: %.1f)\n", SINE_CMD_FREQ);
    printf("  --udp-address IP     UDP address for MAVLink (default: 127.0.0.1)\n");
    printf("  --udp-port PORT      UDP port for MAVLink (default: 14540)\n");
    printf("  --target-system ID   Target system ID (default: 1)\n");
    printf("  --target-component ID Target component ID (default: 1)\n");
    printf("  --frame FRAME        MAV_FRAME value (default: 1 = LOCAL_NED)\n");
    printf("  -h, --help           Show this help message\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s --sine                                    # Sine test via ZMQ\n", program_name);
    printf("  %s --log log.csv                            # Replay log via ZMQ\n", program_name);
    printf("  %s --sine --direct-mavlink                  # Sine test via direct MAVLink\n", program_name);
    printf("  %s --log log.csv --loop --speed 2.0         # Replay log at 2x speed, looping\n", program_name);
}

// Parse command line arguments
void parse_args(int argc, char* argv[], options_t* opts) {
    // Initialize defaults
    memset(opts, 0, sizeof(options_t));
    opts->use_sine = false;
    opts->use_zmq = true;
    opts->loop = false;
    opts->speed = 1.0;
    opts->sine_duration = SINE_DURATION;
    opts->sine_cmd_freq = SINE_CMD_FREQ;
    opts->udp_address = "127.0.0.1";
    opts->udp_port = 14540;
    opts->target_system = 1;
    opts->target_component = 1;
    opts->frame = 1;  // MAV_FRAME_LOCAL_NED
    
    static struct option long_options[] = {
        {"log", required_argument, 0, 'l'},
        {"sine", no_argument, 0, 's'},
        {"direct-mavlink", no_argument, 0, 'd'},
        {"loop", no_argument, 0, 'L'},
        {"speed", required_argument, 0, 'S'},
        {"sine-duration", required_argument, 0, 'D'},
        {"sine-cmd-freq", required_argument, 0, 'F'},
        {"udp-address", required_argument, 0, 'a'},
        {"udp-port", required_argument, 0, 'p'},
        {"target-system", required_argument, 0, 't'},
        {"target-component", required_argument, 0, 'c'},
        {"frame", required_argument, 0, 'f'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    int option_index = 0;
    
    while ((opt = getopt_long(argc, argv, "h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'l':
                opts->logfile = strdup(optarg);
                break;
            case 's':
                opts->use_sine = true;
                break;
            case 'd':
                opts->use_zmq = false;
                break;
            case 'L':
                opts->loop = true;
                break;
            case 'S':
                opts->speed = atof(optarg);
                break;
            case 'D':
                opts->sine_duration = atof(optarg);
                break;
            case 'F':
                opts->sine_cmd_freq = atof(optarg);
                break;
            case 'a':
                opts->udp_address = strdup(optarg);
                break;
            case 'p':
                opts->udp_port = atoi(optarg);
                break;
            case 't':
                opts->target_system = atoi(optarg);
                break;
            case 'c':
                opts->target_component = atoi(optarg);
                break;
            case 'f':
                opts->frame = atoi(optarg);
                break;
            case 'h':
                print_usage(argv[0]);
                exit(0);
            default:
                print_usage(argv[0]);
                exit(1);
        }
    }
    
    // Default to sine if no log file specified
    if (!opts->logfile && !opts->use_sine) {
        opts->use_sine = true;
    }
}

// Serialize velocity command to binary format
void serialize_vel_cmd(double vx, double vy, double vz, double yaw, double yaw_rate, uint8_t* buffer, size_t* len) {
    // Format: magic (4 bytes), version (4 bytes), vel[3] (24 bytes), yaw (8 bytes),
    //         yaw_rate (8 bytes), yaw_valid (1 byte), yaw_rate_valid (1 byte), padding (2 bytes) = 52 bytes total
    uint32_t magic = 0x56454C43;  // "VELC"
    uint32_t version = 1;
    uint8_t yaw_valid = isnan(yaw) ? 0 : 1;
    uint8_t yaw_rate_valid = isnan(yaw_rate) ? 0 : 1;
    double yaw_val = isnan(yaw) ? 0.0 : yaw;
    double yaw_rate_val = isnan(yaw_rate) ? 0.0 : yaw_rate;
    
    size_t offset = 0;
    memcpy(buffer + offset, &magic, 4);
    offset += 4;
    memcpy(buffer + offset, &version, 4);
    offset += 4;
    memcpy(buffer + offset, &vx, 8);
    offset += 8;
    memcpy(buffer + offset, &vy, 8);
    offset += 8;
    memcpy(buffer + offset, &vz, 8);
    offset += 8;
    memcpy(buffer + offset, &yaw_val, 8);
    offset += 8;
    memcpy(buffer + offset, &yaw_rate_val, 8);
    offset += 8;
    buffer[offset++] = yaw_valid;
    buffer[offset++] = yaw_rate_valid;
    buffer[offset++] = 0;  // padding
    buffer[offset++] = 0;  // padding
    
    *len = offset;
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
        // Not a full address string, treat as port number
        *port = atoi(address);
        *is_server_mode = true;
        return 0;
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

// Create MAVLink UDP connection (supports both server and client mode)
mavlink_connection_t* mavlink_connect(const char* address, int port) {
    char ip[64];
    int conn_port;
    bool is_server_mode;
    
    // Parse address string if provided, otherwise use port parameter
    if (address != NULL && strncmp(address, "udp:", 4) == 0) {
        if (parse_mavlink_address(address, ip, sizeof(ip), &conn_port, &is_server_mode) != 0) {
            fprintf(stderr, "Failed to parse MAVLink address: %s\n", address);
            return NULL;
        }
    } else {
        // Use port parameter directly, default to server mode (like hardware_adapter)
        conn_port = port;
        is_server_mode = true;  // Default to server mode for better compatibility
        strncpy(ip, "127.0.0.1", sizeof(ip));
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
    
    // Enable SO_REUSEADDR
    int reuse = 1;
    setsockopt(conn->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    // Set socket to non-blocking
    int flags = fcntl(conn->fd, F_GETFL, 0);
    if (flags < 0 || fcntl(conn->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "Failed to set socket to non-blocking: %s\n", strerror(errno));
        close(conn->fd);
        free(conn);
        return NULL;
    }
    
    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms
    setsockopt(conn->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // Set up local address
    memset(&conn->local_addr, 0, sizeof(conn->local_addr));
    conn->local_addr.sin_family = AF_INET;
    conn->local_addr.sin_addr.s_addr = INADDR_ANY;
    if (is_server_mode) {
        conn->local_addr.sin_port = htons(conn_port);  // Bind to specified port (server mode)
        printf("MAVLink server mode: listening on port %d\n", conn_port);
    } else {
        conn->local_addr.sin_port = 0;  // Let system choose port (client mode)
    }
    
    // Set up remote address (where we send messages)
    // For server mode, remote address will be set from first received message
    // For client mode, set it to the specified IP:PORT
    memset(&conn->remote_addr, 0, sizeof(conn->remote_addr));
    if (!is_server_mode) {
        conn->remote_addr.sin_family = AF_INET;
        conn->remote_addr.sin_port = htons(conn_port);
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
    
    // Initialize MAVLink connection parameters
    conn->system_id = MAV_SYSTEM_ID;
    conn->component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    conn->target_system = 1;  // Will be updated when we receive heartbeat
    conn->target_component = MAV_COMP_ID_AUTOPILOT1;
    conn->connected = false;
    memset(&conn->status, 0, sizeof(conn->status));
    
    if (is_server_mode) {
        printf("MAVLink listening on port %d (server mode) - waiting for messages from drone\n", conn_port);
    } else {
        printf("MAVLink client mode: will send to %s:%d\n", ip, conn_port);
    }
    
    return conn;
}

// Disconnect MAVLink
void mavlink_disconnect(mavlink_connection_t* conn) {
    if (conn == NULL) {
        return;
    }
    
    if (conn->fd >= 0) {
        close(conn->fd);
        conn->fd = -1;
    }
    free(conn);
}

// Send heartbeat
int mavlink_send_heartbeat(mavlink_connection_t* conn) {
    if (conn == NULL || conn->fd < 0) {
        return -1;
    }
    
    // In server mode, don't send until we've received a message (to know where to send)
    if (conn->remote_addr.sin_addr.s_addr == INADDR_ANY || conn->remote_addr.sin_port == 0) {
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
    
    mavlink_msg_heartbeat_encode(conn->system_id, conn->component_id, &msg, &heartbeat);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    ssize_t sent = sendto(conn->fd, buffer, len, 0,
                         (struct sockaddr*)&conn->remote_addr, sizeof(conn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

// Wait for heartbeat
int mavlink_wait_heartbeat(mavlink_connection_t* conn, int timeout_ms) {
    if (conn == NULL || conn->fd < 0) {
        return -1;
    }
    
    struct pollfd pfd;
    pfd.fd = conn->fd;
    pfd.events = POLLIN;
    
    int poll_result = poll(&pfd, 1, timeout_ms);
    if (poll_result <= 0) {
        return -1;
    }
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    
    ssize_t recv_len = recvfrom(conn->fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                                (struct sockaddr*)&src_addr, &addr_len);
    
    if (recv_len < 0) {
        return -1;
    }
    
    // Update remote address from first received message
    if (conn->remote_addr.sin_addr.s_addr == INADDR_ANY || conn->remote_addr.sin_port == 0) {
        memcpy(&conn->remote_addr, &src_addr, sizeof(src_addr));
    }
    
    // Parse MAVLink message
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (ssize_t i = 0; i < recv_len; i++) {
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status);
        
        if (result == MAVLINK_FRAMING_OK && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            conn->status = status;
            conn->target_system = msg.sysid;
            conn->target_component = msg.compid;
            conn->connected = true;
            return 0;
        }
    }
    
    return -1;
}

// Receive heartbeat (non-blocking)
int mavlink_receive_heartbeat(mavlink_connection_t* conn, uint32_t* custom_mode) {
    if (conn == NULL || conn->fd < 0) {
        return -1;
    }
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    
    ssize_t recv_len = recvfrom(conn->fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                                (struct sockaddr*)&src_addr, &addr_len);
    
    if (recv_len < 0) {
        return -1;
    }
    
    // Update remote address
    if (conn->remote_addr.sin_addr.s_addr == INADDR_ANY || conn->remote_addr.sin_port == 0) {
        memcpy(&conn->remote_addr, &src_addr, sizeof(src_addr));
    }
    
    // Parse MAVLink message
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (ssize_t i = 0; i < recv_len; i++) {
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status);
        
        if (result == MAVLINK_FRAMING_OK && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            conn->status = status;
            conn->target_system = msg.sysid;
            conn->target_component = msg.compid;
            conn->connected = true;
            if (custom_mode != NULL) {
                *custom_mode = heartbeat.custom_mode;
            }
            return 0;
        }
    }
    
    return -1;
}

// Send SET_POSITION_TARGET_LOCAL_NED
int mavlink_send_set_position_target_local_ned(mavlink_connection_t* conn, uint32_t time_boot_ms,
                                                uint8_t coordinate_frame, uint16_t type_mask,
                                                float pos_x, float pos_y, float pos_z,
                                                float vel_x, float vel_y, float vel_z,
                                                float acc_x, float acc_y, float acc_z,
                                                float yaw, float yaw_rate) {
    if (conn == NULL || conn->fd < 0) {
        return -1;
    }
    
    // In server mode, don't send until we've received a message (to know where to send)
    if (conn->remote_addr.sin_addr.s_addr == INADDR_ANY || conn->remote_addr.sin_port == 0) {
        return -1;  // No remote address set yet
    }
    
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t setpoint;
    
    setpoint.time_boot_ms = time_boot_ms;
    setpoint.target_system = conn->target_system;
    setpoint.target_component = conn->target_component;
    setpoint.coordinate_frame = coordinate_frame;
    setpoint.type_mask = type_mask;
    setpoint.x = pos_x;
    setpoint.y = pos_y;
    setpoint.z = pos_z;
    setpoint.vx = vel_x;
    setpoint.vy = vel_y;
    setpoint.vz = vel_z;
    setpoint.afx = acc_x;
    setpoint.afy = acc_y;
    setpoint.afz = acc_z;
    setpoint.yaw = yaw;
    setpoint.yaw_rate = yaw_rate;
    
    mavlink_msg_set_position_target_local_ned_encode(conn->system_id, conn->component_id, &msg, &setpoint);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    ssize_t sent = sendto(conn->fd, buffer, len, 0,
                         (struct sockaddr*)&conn->remote_addr, sizeof(conn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

// Send COMMAND_LONG
int mavlink_send_command_long(mavlink_connection_t* conn, uint16_t command,
                              float param1, float param2, float param3,
                              float param4, float param5, float param6, float param7) {
    if (conn == NULL || conn->fd < 0) {
        return -1;
    }
    
    // In server mode, don't send until we've received a message (to know where to send)
    if (conn->remote_addr.sin_addr.s_addr == INADDR_ANY || conn->remote_addr.sin_port == 0) {
        return -1;  // No remote address set yet
    }
    
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    
    cmd.target_system = conn->target_system;
    cmd.target_component = conn->target_component;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;
    
    mavlink_msg_command_long_encode(conn->system_id, conn->component_id, &msg, &cmd);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    ssize_t sent = sendto(conn->fd, buffer, len, 0,
                         (struct sockaddr*)&conn->remote_addr, sizeof(conn->remote_addr));
    
    return (sent == len) ? 0 : -1;
}

// Set mode to OFFBOARD
void set_mode_offboard(mavlink_connection_t* conn) {
    if (conn == NULL) {
        return;
    }
    
    printf("Setting mode to OFFBOARD...\n");
    
    // Send command multiple times
    for (int i = 0; i < 5; i++) {
        mavlink_send_command_long(conn, 176, 1.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        usleep(100000);  // 100ms
    }
    
    // Wait for confirmation
    printf("Waiting for OFFBOARD mode confirmation...\n");
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    double timeout = 5.0;
    
    while (g_running) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed >= timeout) {
            break;
        }
        
        uint32_t custom_mode = 0;
        if (mavlink_receive_heartbeat(conn, &custom_mode) == 0) {
            if (custom_mode == PX4_FLIGHT_STATE_OFFBOARD) {
                printf("✓ OFFBOARD mode confirmed!\n");
                return;
            }
        }
        usleep(100000);  // 100ms
    }
    
    printf("Warning: OFFBOARD mode not confirmed after 5 seconds\n");
}

// Set mode to HOLD
void set_mode_hold(mavlink_connection_t* conn) {
    if (conn == NULL) {
        return;
    }
    
    printf("Setting mode to HOLD...\n");
    
    // Send command multiple times
    for (int i = 0; i < 5; i++) {
        mavlink_send_command_long(conn, 176, 1.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        usleep(100000);  // 100ms
    }
    
    // Wait for confirmation
    printf("Waiting for HOLD mode confirmation...\n");
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    double timeout = 5.0;
    
    while (g_running) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed >= timeout) {
            break;
        }
        
        uint32_t custom_mode = 0;
        if (mavlink_receive_heartbeat(conn, &custom_mode) == 0) {
            if (custom_mode == PX4_FLIGHT_STATE_HOLD) {
                printf("✓ HOLD mode confirmed!\n");
                return;
            }
        }
        usleep(100000);  // 100ms
    }
    
    printf("Warning: HOLD mode not confirmed after 5 seconds\n");
}

// Replay log file
void replay_log_once(const char* logfile, double speed, bool use_zmq) {
    FILE* fp = fopen(logfile, "r");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open log file: %s\n", logfile);
        return;
    }
    
    // Read header
    char line[2048];
    if (fgets(line, sizeof(line), fp) == NULL) {
        fclose(fp);
        return;
    }
    
    // Parse header to find column indices
    char* token;
    char* saveptr;
    int col_timestamp = -1, col_local_ts = -1, col_cmd_type = -1;
    int col_pos_x = -1, col_pos_y = -1, col_pos_z = -1;
    int col_vel_x = -1, col_vel_y = -1, col_vel_z = -1;
    int col_acc_x = -1, col_acc_y = -1, col_acc_z = -1;
    int col_yaw = -1, col_yaw_rate = -1;
    
    int col_idx = 0;
    token = strtok_r(line, ",\n", &saveptr);
    while (token != NULL) {
        if (strcmp(token, "timestamp") == 0) col_timestamp = col_idx;
        else if (strcmp(token, "local_ts") == 0) col_local_ts = col_idx;
        else if (strcmp(token, "command_type") == 0) col_cmd_type = col_idx;
        else if (strcmp(token, "pos_x") == 0) col_pos_x = col_idx;
        else if (strcmp(token, "pos_y") == 0) col_pos_y = col_idx;
        else if (strcmp(token, "pos_z") == 0) col_pos_z = col_idx;
        else if (strcmp(token, "vel_x") == 0) col_vel_x = col_idx;
        else if (strcmp(token, "vel_y") == 0) col_vel_y = col_idx;
        else if (strcmp(token, "vel_z") == 0) col_vel_z = col_idx;
        else if (strcmp(token, "acc_x") == 0) col_acc_x = col_idx;
        else if (strcmp(token, "acc_y") == 0) col_acc_y = col_idx;
        else if (strcmp(token, "acc_z") == 0) col_acc_z = col_idx;
        else if (strcmp(token, "yaw") == 0) col_yaw = col_idx;
        else if (strcmp(token, "yaw_rate") == 0) col_yaw_rate = col_idx;
        col_idx++;
        token = strtok_r(NULL, ",\n", &saveptr);
    }
    
    // Check required columns
    if (col_timestamp < 0 || col_local_ts < 0 || col_cmd_type < 0 ||
        col_vel_x < 0 || col_vel_y < 0 || col_vel_z < 0) {
        fprintf(stderr, "Log file missing required columns\n");
        fclose(fp);
        return;
    }
    
    double prev_local_ts = -1.0;
    struct timespec start_wall_time;
    bool first_row = true;
    int sent_count = 0;
    struct timespec last_print_time;
    clock_gettime(CLOCK_MONOTONIC, &last_print_time);
    
    // Read and process rows
    while (g_running && fgets(line, sizeof(line), fp) != NULL) {
        // Parse CSV row
        char* values[64];
        int num_values = 0;
        token = strtok_r(line, ",\n", &saveptr);
        while (token != NULL && num_values < 64) {
            values[num_values++] = token;
            token = strtok_r(NULL, ",\n", &saveptr);
        }
        
        // Check command type
        if (col_cmd_type >= 0 && num_values > col_cmd_type) {
            if (strcmp(values[col_cmd_type], "SETPOINT") != 0) {
                continue;
            }
        }
        
        // Parse values
        double timestamp_s = atof(values[col_timestamp]);
        double local_ts = atof(values[col_local_ts]);
        float pos_x = (col_pos_x >= 0 && num_values > col_pos_x) ? atof(values[col_pos_x]) : 0.0f;
        float pos_y = (col_pos_y >= 0 && num_values > col_pos_y) ? atof(values[col_pos_y]) : 0.0f;
        float pos_z = (col_pos_z >= 0 && num_values > col_pos_z) ? atof(values[col_pos_z]) : 0.0f;
        float vel_x = atof(values[col_vel_x]);
        float vel_y = atof(values[col_vel_y]);
        float vel_z = atof(values[col_vel_z]);
        float acc_x = (col_acc_x >= 0 && num_values > col_acc_x) ? atof(values[col_acc_x]) : 0.0f;
        float acc_y = (col_acc_y >= 0 && num_values > col_acc_y) ? atof(values[col_acc_y]) : 0.0f;
        float acc_z = (col_acc_z >= 0 && num_values > col_acc_z) ? atof(values[col_acc_z]) : 0.0f;
        float yaw = (col_yaw >= 0 && num_values > col_yaw) ? atof(values[col_yaw]) : 0.0f;
        float yaw_rate = (col_yaw_rate >= 0 && num_values > col_yaw_rate) ? atof(values[col_yaw_rate]) : 0.0f;
        
        uint32_t time_boot_ms = (uint32_t)(timestamp_s * 1000.0);
        
        // Timing control
        if (speed > 0.0) {
            if (first_row) {
                prev_local_ts = local_ts;
                clock_gettime(CLOCK_MONOTONIC, &start_wall_time);
                first_row = false;
            } else {
                double dt_log = local_ts - prev_local_ts;
                if (dt_log > 0) {
                    double sleep_time = dt_log / speed;
                    if (sleep_time > 0) {
                        usleep((useconds_t)(sleep_time * 1e6));
                    }
                }
                prev_local_ts = local_ts;
            }
        } else {
            prev_local_ts = local_ts;
        }
        
        // Build type mask (ignore position and acceleration, use velocity and yaw/yaw_rate)
        uint16_t type_mask = 0x07 | 0x1C0;  // ignore pos, acc
        
        if (use_zmq) {
            // Send via ZMQ (using zmq_publisher_send which handles topic+data format)
            uint8_t cmd_data[64];
            size_t cmd_len;
            serialize_vel_cmd(vel_x, vel_y, vel_z, yaw, yaw_rate, cmd_data, &cmd_len);
            if (zmq_publisher_send(g_zmq_pub, TOPIC_GUIDANCE_CMD_VEL_NED, cmd_data, cmd_len) != 0) {
                fprintf(stderr, "Warning: Failed to send ZMQ message\n");
            }
        } else {
            // Send via MAVLink (only if remote address is set)
            if (g_mavlink_conn->remote_addr.sin_addr.s_addr != INADDR_ANY && 
                g_mavlink_conn->remote_addr.sin_port != 0) {
                int result = mavlink_send_set_position_target_local_ned(g_mavlink_conn, time_boot_ms, 1, type_mask,
                                                                          pos_x, pos_y, pos_z,
                                                                          vel_x, vel_y, vel_z,
                                                                          acc_x, acc_y, acc_z,
                                                                          yaw, yaw_rate);
                // If send failed, try to update remote address
                if (result != 0 && sent_count % 100 == 0) {
                    uint32_t custom_mode = 0;
                    mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
                }
            } else {
                // Try to receive message to set remote address
                uint32_t custom_mode = 0;
                mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
            }
        }
        
        sent_count++;
        
        // Periodically print status
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - last_print_time.tv_sec) + (now.tv_nsec - last_print_time.tv_nsec) / 1e9;
        if (elapsed >= 1.0) {
            printf("Replay cmd #%d: vel=(%.3f, %.3f, %.3f) m/s, yaw=%.3f rad, yaw_rate=%.3f rad/s\n",
                   sent_count, vel_x, vel_y, vel_z, yaw, yaw_rate);
            last_print_time = now;
        }
        
        // Periodically send heartbeat and check for incoming messages (MAVLink only)
        if (!use_zmq && sent_count % 10 == 0) {
            // Check for incoming heartbeats first to update target system/component
            uint32_t custom_mode = 0;
            mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
            // Send heartbeat
            mavlink_send_heartbeat(g_mavlink_conn);
        }
    }
    
    fclose(fp);
    printf("Finished replaying log, sent %d SETPOINT messages\n", sent_count);
}

// Run sine test
void run_sine_test(double duration, double cmd_freq, bool use_zmq) {
    double dt = 1.0 / cmd_freq;
    
    printf("Starting sine test with amplitudes vx=%.1f m/s, vy=%.1f m/s, vz=%.1f m/s, yaw_rate=%.1f rad/s\n",
           SINE_VX_AMPL, SINE_VY_AMPL, SINE_VZ_AMPL, SINE_YAWRATE_AMPL);
    printf("Command frequency: %.1f Hz (dt = %.2f ms)\n", cmd_freq, dt * 1000.0);
    
    uint16_t type_mask = 0x07 | 0x1C0 | 0x400;  // ignore pos, acc, yaw; use vel and yaw_rate
    
    struct timespec start_time, now, last_heartbeat_time, last_print_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    last_heartbeat_time = start_time;
    last_print_time = start_time;
    int sent_count = 0;
    
    while (g_running) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double t = (now.tv_sec - start_time.tv_sec) + (now.tv_nsec - start_time.tv_nsec) / 1e9;
        
        if (duration > 0 && t >= duration) {
            break;
        }
        
        // Compute sine values
        double vx = SINE_VX_AMPL * sin(2.0 * M_PI * SINE_VX_FREQ * t);
        double vy = SINE_VY_AMPL * sin(2.0 * M_PI * SINE_VY_FREQ * t);
        double vz = SINE_VZ_AMPL * sin(2.0 * M_PI * SINE_VZ_FREQ * t);
        double yaw_rate = SINE_YAWRATE_AMPL * sin(2.0 * M_PI * SINE_YAWRATE_FREQ * t);
        
        uint32_t time_boot_ms = (uint32_t)(t * 1000.0);
        
        if (use_zmq) {
            // Send via ZMQ (using zmq_publisher_send which handles topic+data format)
            uint8_t cmd_data[64];
            size_t cmd_len;
            serialize_vel_cmd(vx, vy, vz, NAN, yaw_rate, cmd_data, &cmd_len);
            if (zmq_publisher_send(g_zmq_pub, TOPIC_GUIDANCE_CMD_VEL_NED, cmd_data, cmd_len) != 0) {
                fprintf(stderr, "Warning: Failed to send ZMQ message\n");
            }
        } else {
            // Send via MAVLink (only if remote address is set)
            if (g_mavlink_conn->remote_addr.sin_addr.s_addr != INADDR_ANY && 
                g_mavlink_conn->remote_addr.sin_port != 0) {
                int result = mavlink_send_set_position_target_local_ned(g_mavlink_conn, time_boot_ms, 1, type_mask,
                                                                         0.0f, 0.0f, 0.0f,
                                                                         vx, vy, vz,
                                                                         0.0f, 0.0f, 0.0f,
                                                                         0.0f, yaw_rate);
                // If send failed, try to update remote address
                if (result != 0 && sent_count % 100 == 0) {
                    uint32_t custom_mode = 0;
                    mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
                }
            } else {
                // Try to receive message to set remote address
                uint32_t custom_mode = 0;
                mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
            }
        }
        
        sent_count++;
        
        // Periodically print status
        double elapsed = (now.tv_sec - last_print_time.tv_sec) + (now.tv_nsec - last_print_time.tv_nsec) / 1e9;
        if (elapsed >= 1.0) {
            printf("Sine cmd #%d: vel=(%.3f, %.3f, %.3f) m/s, yaw_rate=%.3f rad/s\n",
                   sent_count, vx, vy, vz, yaw_rate);
            last_print_time = now;
        }
        
        // Periodically send heartbeat and check for incoming messages (MAVLink only)
        if (!use_zmq) {
            double hb_elapsed = (now.tv_sec - last_heartbeat_time.tv_sec) + (now.tv_nsec - last_heartbeat_time.tv_nsec) / 1e9;
            if (hb_elapsed >= 1.0) {
                mavlink_send_heartbeat(g_mavlink_conn);
                // Check for incoming heartbeats to update target system/component
                uint32_t custom_mode = 0;
                if (mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode) == 0 && sent_count % 50 == 0) {
                    printf("Sine: sent %d messages, current mode: %u\n", sent_count, custom_mode);
                }
                last_heartbeat_time = now;
            }
        }
        
        // Sleep according to desired update rate
        usleep((useconds_t)(dt * 1e6));
    }
    
    printf("Sine test finished, sent %d SETPOINT messages\n", sent_count);
}

// Main function
int main(int argc, char* argv[]) {
    options_t opts;
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Parse arguments
    parse_args(argc, argv, &opts);
    
    // Initialize ZMQ if needed
    if (opts.use_zmq) {
        zmq_wrapper_init();
        g_zmq_pub = zmq_publisher_create(TOPIC_GUIDANCE_CMD_PORT);
        if (g_zmq_pub == NULL) {
            fprintf(stderr, "Failed to create ZMQ publisher\n");
            return 1;
        }
        printf("Sending commands via ZMQ on port %d, topic %s\n",
               TOPIC_GUIDANCE_CMD_PORT, TOPIC_GUIDANCE_CMD_VEL_NED);
        usleep(50000);  // 50ms to ensure connection
    } else {
        // Initialize MAVLink connection
        // Use server mode (listen on port) like hardware_adapter - this allows receiving from QGroundControl/drone
        char address_str[128];
        snprintf(address_str, sizeof(address_str), "udp:%d", opts.udp_port);
        g_mavlink_conn = mavlink_connect(address_str, opts.udp_port);
        if (g_mavlink_conn == NULL) {
            fprintf(stderr, "Failed to create MAVLink connection\n");
            return 1;
        }
        
        // Wait for heartbeat from autopilot (this sets remote address in server mode)
        printf("Waiting for heartbeat from autopilot (listening on port %d)...\n", opts.udp_port);
        int heartbeat_received = 0;
        for (int attempt = 0; attempt < 20; attempt++) {
            if (mavlink_wait_heartbeat(g_mavlink_conn, 250) == 0) {
                printf("Heartbeat received from system %d, component %d\n",
                       g_mavlink_conn->target_system, g_mavlink_conn->target_component);
                heartbeat_received = 1;
                break;
            }
            // Try to receive any message to set remote address (non-blocking)
            uint32_t custom_mode = 0;
            mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode);
        }
        
        if (!heartbeat_received) {
            printf("Warning: No heartbeat received after 5 seconds.\n");
            printf("Make sure QGroundControl/drone is sending MAVLink messages to port %d\n", opts.udp_port);
            printf("Continuing anyway - remote address will be set from first received message.\n");
        }
        
        // Send initial heartbeat now that we might have remote address
        if (g_mavlink_conn->remote_addr.sin_addr.s_addr != INADDR_ANY && 
            g_mavlink_conn->remote_addr.sin_port != 0) {
            for (int i = 0; i < 3; i++) {
                mavlink_send_heartbeat(g_mavlink_conn);
                usleep(100000);  // 100ms
            }
        }
        
        // Set mode to OFFBOARD (only if we have remote address)
        if (g_mavlink_conn->remote_addr.sin_addr.s_addr != INADDR_ANY && 
            g_mavlink_conn->remote_addr.sin_port != 0) {
            set_mode_offboard(g_mavlink_conn);
        } else {
            printf("Warning: Cannot set OFFBOARD mode yet - waiting for first message to set remote address...\n");
            // Wait a bit more for first message
            for (int i = 0; i < 20; i++) {
                uint32_t custom_mode = 0;
                if (mavlink_receive_heartbeat(g_mavlink_conn, &custom_mode) == 0) {
                    printf("Received first message, remote address set. Setting OFFBOARD mode...\n");
                    // Send heartbeats first
                    for (int j = 0; j < 3; j++) {
                        mavlink_send_heartbeat(g_mavlink_conn);
                        usleep(100000);
                    }
                    set_mode_offboard(g_mavlink_conn);
                    break;
                }
                usleep(100000);  // 100ms
            }
        }
    }
    
    // Run test
    if (opts.use_sine) {
        run_sine_test(opts.sine_duration, opts.sine_cmd_freq, opts.use_zmq);
    } else {
        do {
            replay_log_once(opts.logfile, opts.speed, opts.use_zmq);
        } while (opts.loop && g_running);
    }
    
    // Cleanup
    if (!opts.use_zmq && g_mavlink_conn != NULL) {
        set_mode_hold(g_mavlink_conn);
        mavlink_disconnect(g_mavlink_conn);
        g_mavlink_conn = NULL;
    }
    
    if (g_zmq_pub != NULL) {
        zmq_publisher_destroy(g_zmq_pub);
        g_zmq_pub = NULL;
        zmq_wrapper_cleanup();
    }
    
    if (opts.logfile) {
        free(opts.logfile);
    }
    if (opts.udp_address && strcmp(opts.udp_address, "127.0.0.1") != 0) {
        free(opts.udp_address);
    }
    
    return 0;
}
