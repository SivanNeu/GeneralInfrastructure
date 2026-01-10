#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "serial_to_ZMQ.h"
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
static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size, void* msg_dict, size_t msg_dict_size);
static int mavlink_send_heartbeat(void* conn);
static int mavlink_wait_heartbeat(void* conn, int timeout_ms);
static const char* mavlink_get_message_name_by_id(uint32_t msgid);

// Serial port helper functions
static int serial_open_port(const char* device);
static bool serial_setup_port(int fd, int baudrate);
static ssize_t serial_read_port(int fd, uint8_t* buffer, size_t buffer_size);
static ssize_t serial_write_port(int fd, const uint8_t* buffer, size_t buffer_size);

// Serialization helper (simplified - in production use proper serialization like msgpack or protobuf)
static size_t flight_data_serialize(const flight_data_t* fd, void* buffer, size_t buffer_size);
static int flight_data_deserialize(flight_data_t* fd, const void* buffer, size_t buffer_size);

// Initialize serial to ZMQ bridge
int serial_to_zmq_init(serial_to_zmq_t* bridge, const char* log_dir) {
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
    
    // Initialize mutex
    if (pthread_mutex_init(&bridge->data_lock, NULL) != 0) {
        fprintf(stderr, "Failed to initialize data mutex\n");
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Initialize filters
    lpf_init(&bridge->vertical_speed_filter, 0.1, false, LPF_TYPE_FIRST_ORDER);
    lpf_init(&bridge->altitude_filter, 0.3, false, LPF_TYPE_FIRST_ORDER);
    bridge->prev_alt_m = -1.0;  // Invalid value to indicate not initialized
    bridge->alt_vel_count = 0;
    bridge->prev_vel_vertical = 0.0;
    bridge->prev_alt_ts = 0.0;
    
    // Initialize flight data
    flight_data_init(&bridge->current_data);
    
    // Initialize ZMQ
    zmq_wrapper_init();
    
    // Initialize mavlink
    // Use serial port default if not set
    if (bridge->mavlink_address == NULL) {
        bridge->mavlink_address = strdup("/dev/ttyS0:921600");
        if (bridge->mavlink_address == NULL) {
            fprintf(stderr, "Failed to allocate memory for mavlink_address\n");
            pthread_mutex_destroy(&bridge->data_lock);
            free(bridge->log_dir);
            bridge->log_dir = NULL;
            return -1;
        }
    }
    if (serial_to_zmq_init_mavlink(bridge) != 0) {
        fprintf(stderr, "Failed to initialize mavlink connection\n");
        bridge->init_success = false;
        free(bridge->mavlink_address);
        bridge->mavlink_address = NULL;
        pthread_mutex_destroy(&bridge->data_lock);
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Create ZMQ publisher socket
    bridge->pub_socket = zmq_publisher_create(TOPIC_MAVLINK_PORT);
    if (bridge->pub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ publisher socket\n");
        bridge->init_success = false;
        mavlink_disconnect(bridge->mavlink_connection);
        bridge->mavlink_connection = NULL;
        free(bridge->mavlink_address);
        bridge->mavlink_address = NULL;
        pthread_mutex_destroy(&bridge->data_lock);
        free(bridge->log_dir);
        bridge->log_dir = NULL;
        return -1;
    }
    
    // Wait a bit for publisher to bind
    usleep(200000);  // 200ms
    
    // Initialize control flags
    bridge->mavlink_connected_to_usb = false;
    bridge->disable_offboard_control = false;
    bridge->offboard_control_enabled = false;
    bridge->running = true;
    bridge->init_success = true;
    
    // Start data thread
    if (pthread_create(&bridge->data_thread, NULL, serial_to_zmq_data_thread_func, bridge) != 0) {
        fprintf(stderr, "Failed to create data thread\n");
        bridge->init_success = false;
        return -1;
    }
    
    printf("Serial to ZMQ bridge started successfully\n");
    printf("Publishing to topic: %s on port: %d\n", TOPIC_MAVLINK_FLIGHT_DATA, TOPIC_MAVLINK_PORT);
    
    return 0;
}

void serial_to_zmq_cleanup(serial_to_zmq_t* bridge) {
    if (bridge == NULL) {
        return;
    }
    
    // Stop threads
    serial_to_zmq_stop(bridge);
    
    // Close ZMQ sockets
    if (bridge->pub_socket != NULL) {
        zmq_publisher_destroy(bridge->pub_socket);
        bridge->pub_socket = NULL;
    }
    
    // Disconnect mavlink
    if (bridge->mavlink_connection != NULL) {
        mavlink_disconnect(bridge->mavlink_connection);
        bridge->mavlink_connection = NULL;
    }
    
    // Cleanup flight data
    flight_data_cleanup(&bridge->current_data);
    
    // Destroy mutex
    pthread_mutex_destroy(&bridge->data_lock);
    
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

bool serial_to_zmq_init_succeeded(serial_to_zmq_t* bridge) {
    return bridge != NULL && bridge->init_success;
}

void serial_to_zmq_stop(serial_to_zmq_t* bridge) {
    if (bridge == NULL) {
        return;
    }
    
    bridge->running = false;
    
    // Wait for threads to finish
    if (bridge->data_thread != 0) {
        pthread_join(bridge->data_thread, NULL);
    }
}

// Initialize mavlink connection
int serial_to_zmq_init_mavlink(serial_to_zmq_t* bridge) {
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
        if (mavlink_wait_heartbeat(bridge->mavlink_connection, 2000) == 0) {
            printf("MAVLink heartbeat received\n");
            return 0;
        }
    }
    
    // Connection established, but no heartbeat yet - this is OK
    // The heartbeat will come when the MAVLink device is running
    printf("MAVLink connection established (waiting for heartbeat from device)\n");
    return 0;
}

// Listen to mavlink messages
void serial_to_zmq_listener_to_mavlink(serial_to_zmq_t* bridge, bool blocking, double timeout, 
                                          bool use_lock, bool apply_filter) {
    char msg_type[64];
    char msg_dict[1024];  // Simplified - in production use proper message structure
    
    int timeout_ms = blocking ? (int)(timeout * 1000) : 0;
    int result = mavlink_receive_message(bridge->mavlink_connection, msg_type, sizeof(msg_type),
                                        msg_dict, sizeof(msg_dict));
    
    if (result < 0) {
        return;  // No message or error
    }
    
    // Parse message
    if (use_lock) {
        pthread_mutex_lock(&bridge->data_lock);
        serial_to_zmq_parse(bridge, msg_type, msg_dict);
        if (apply_filter) {
            serial_to_zmq_filter_data(bridge, &bridge->current_data);
        }
        pthread_mutex_unlock(&bridge->data_lock);
    } else {
        serial_to_zmq_parse(bridge, msg_type, msg_dict);
        if (apply_filter) {
            serial_to_zmq_filter_data(bridge, &bridge->current_data);
        }
    }
}

// Filter data
void serial_to_zmq_filter_data(serial_to_zmq_t* bridge, flight_data_t* current_data) {
    double vel_vertical = 0.0;
    
    if (bridge->prev_alt_m >= 0.0) {  // Valid previous altitude
        double dt = current_data->altitude_m.timestamp - bridge->prev_alt_ts;
        if (dt != 0.0) {
            current_data->altitude_m.relative = lpf_step(&bridge->altitude_filter, 
                                                          current_data->altitude_m.relative);
            double d_alt = current_data->altitude_m.relative - bridge->prev_alt_m;
            vel_vertical = d_alt / dt;
            
            if (fabs(vel_vertical - bridge->prev_vel_vertical) > MAX_VERTICAL_VEL_JUMP_M_S) {
                if (vel_vertical - bridge->prev_vel_vertical > 0) {
                    vel_vertical = bridge->prev_vel_vertical + MAX_VERTICAL_VEL_JUMP_M_S;
                } else {
                    vel_vertical = bridge->prev_vel_vertical - MAX_VERTICAL_VEL_JUMP_M_S;
                }
            }
            
            current_data->altitude_m.vertical_speed_estimate = lpf_step(&bridge->vertical_speed_filter, 
                                                                        vel_vertical);
            bridge->prev_vel_vertical = current_data->altitude_m.vertical_speed_estimate;
            bridge->prev_alt_m = current_data->altitude_m.relative;
            bridge->prev_alt_ts = current_data->altitude_m.timestamp;
        }
    } else {
        bridge->prev_alt_m = current_data->altitude_m.relative;
        bridge->prev_alt_ts = current_data->altitude_m.timestamp;
    }
}

// Parse mavlink message
void serial_to_zmq_parse(serial_to_zmq_t* bridge, const char* msg_type, void* msg_dict) {
    if (bridge == NULL || msg_type == NULL || msg_dict == NULL) {
        return;
    }
    
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    bridge->current_data.local_ts = ts.tv_sec + ts.tv_nsec / 1e9;
    
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
            double current_time = bridge->current_data.local_ts;
            
            if (heartbeat.custom_mode != prev_custom_mode_id) {
                mode_change_count++;
                // Print mode change info (limit to avoid spam)
                if (current_time - last_mode_debug_time > 1.0 || mode_change_count <= 5) {
                    printf("serial_to_zmq: Mode change #%d: %d -> %d (OFFBOARD=393216, HOLD=50593792)\n",
                           mode_change_count, prev_custom_mode_id, heartbeat.custom_mode);
                    last_mode_debug_time = current_time;
                }
                prev_custom_mode_id = heartbeat.custom_mode;
            }
            
            bridge->current_data.custom_mode_id = heartbeat.custom_mode;
            bridge->current_data.gathered.custom_mode_id = true;
        }
        // Note: mode string would need to be converted from custom_mode_id
    }
    else if (strcmp(msg_type, "ATTITUDE_QUATERNION") == 0) {
        mavlink_attitude_quaternion_t att_quat;
        mavlink_msg_attitude_quaternion_decode(msg, &att_quat);
        
        quaternion_init(&bridge->current_data.quat_ned_bodyfrd,
                       att_quat.q2, att_quat.q3, att_quat.q4, att_quat.q1);
        bridge->current_data.timestamp = att_quat.time_boot_ms;
        bridge->current_data.rpy_rates.data[0] = att_quat.rollspeed;
        bridge->current_data.rpy_rates.data[1] = att_quat.pitchspeed;
        bridge->current_data.rpy_rates.data[2] = att_quat.yawspeed;
        bridge->current_data.gathered.quat_ned_bodyfrd = true;
        bridge->current_data.gathered.rpy_rates = true;
    }
    else if (strcmp(msg_type, "ATTITUDE") == 0) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        
        bridge->current_data.timestamp = attitude.time_boot_ms;
        bridge->current_data.rpy.data[0] = attitude.roll;
        bridge->current_data.rpy.data[1] = attitude.pitch;
        bridge->current_data.rpy.data[2] = attitude.yaw;
        bridge->current_data.rpy_rates.data[0] = attitude.rollspeed;
        bridge->current_data.rpy_rates.data[1] = attitude.pitchspeed;
        bridge->current_data.rpy_rates.data[2] = attitude.yawspeed;
        bridge->current_data.gathered.rpy = true;
        bridge->current_data.gathered.rpy_rates = true;
    }
    else if (strcmp(msg_type, "LOCAL_POSITION_NED") == 0) {
        mavlink_local_position_ned_t local_pos;
        mavlink_msg_local_position_ned_decode(msg, &local_pos);
        
        bridge->current_data.timestamp = local_pos.time_boot_ms;
        bridge->current_data.pos_ned_m.ned.data[0] = local_pos.x;
        bridge->current_data.pos_ned_m.ned.data[1] = local_pos.y;
        bridge->current_data.pos_ned_m.ned.data[2] = local_pos.z;
        bridge->current_data.pos_ned_m.vel_ned.data[0] = local_pos.vx;
        bridge->current_data.pos_ned_m.vel_ned.data[1] = local_pos.vy;
        bridge->current_data.pos_ned_m.vel_ned.data[2] = local_pos.vz;
        bridge->current_data.pos_ned_m.timestamp = local_pos.time_boot_ms;
        bridge->current_data.gathered.pos_ned_m = true;
        bridge->current_data.gathered.vel_ned_m = true;
    }
    else if (strcmp(msg_type, "GLOBAL_POSITION_INT") == 0) {
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(msg, &global_pos);
        
        bridge->current_data.timestamp = global_pos.time_boot_ms;
        bridge->current_data.filt_pos_lla_deg.lla.data[0] = global_pos.lat / 1e7;
        bridge->current_data.filt_pos_lla_deg.lla.data[1] = global_pos.lon / 1e7;
        bridge->current_data.filt_pos_lla_deg.lla.data[2] = global_pos.alt / 1e3;
        bridge->current_data.relative_m = global_pos.relative_alt / 1000.0;
        bridge->current_data.heading = DEG2RAD(global_pos.hdg / 100.0);
        bridge->current_data.pos_ned_m.vel_ned.data[0] = global_pos.vx / 100.0;
        bridge->current_data.pos_ned_m.vel_ned.data[1] = global_pos.vy / 100.0;
        bridge->current_data.pos_ned_m.vel_ned.data[2] = global_pos.vz / 100.0;
        bridge->current_data.gathered.pos_ned_m = true;
        bridge->current_data.gathered.vel_ned_m = true;
    }
    else if (strcmp(msg_type, "HIGHRES_IMU") == 0) {
        mavlink_highres_imu_t imu;
        mavlink_msg_highres_imu_decode(msg, &imu);
        
        bridge->current_data.timestamp = imu.time_usec / 1000.0;
        bridge->current_data.imu_ned.accel.data[0] = imu.xacc;
        bridge->current_data.imu_ned.accel.data[1] = imu.yacc;
        bridge->current_data.imu_ned.accel.data[2] = imu.zacc;
        bridge->current_data.imu_ned.gyro.data[0] = imu.xgyro;
        bridge->current_data.imu_ned.gyro.data[1] = imu.ygyro;
        bridge->current_data.imu_ned.gyro.data[2] = imu.zgyro;
        bridge->current_data.imu_ned.timestamp = bridge->current_data.timestamp;
        bridge->current_data.absolute_press_hpa = imu.abs_pressure;
        bridge->current_data.differential_press_hpa = imu.diff_pressure;
        bridge->current_data.pressure = imu.pressure_alt;
        bridge->current_data.temperature = imu.temperature;
        bridge->current_data.gathered.imu_ned = true;
        bridge->current_data.gathered.absolute_press_hpa = true;
        bridge->current_data.gathered.differential_press_hpa = true;
        bridge->current_data.gathered.pressure = true;
        bridge->current_data.gathered.temperature = true;
    }
    else if (strcmp(msg_type, "ALTITUDE") == 0) {
        mavlink_altitude_t altitude;
        mavlink_msg_altitude_decode(msg, &altitude);
        
        bridge->current_data.timestamp = altitude.time_usec / 1000.0;
        bridge->current_data.relative_m = altitude.altitude_relative;
        bridge->current_data.amsl_m = altitude.altitude_amsl;
        bridge->current_data.local_m = altitude.altitude_local;
        bridge->current_data.monotonic_m = altitude.altitude_monotonic;
        bridge->current_data.terrain_m = altitude.altitude_terrain;
        bridge->current_data.bottom_clearance_m = altitude.bottom_clearance;
        bridge->current_data.altitude_m.relative = altitude.altitude_relative;
        bridge->current_data.altitude_m.amsl = altitude.altitude_amsl;
        bridge->current_data.altitude_m.timestamp = bridge->current_data.timestamp;
        bridge->current_data.gathered.relative_m = true;
        bridge->current_data.gathered.amsl_m = true;
        bridge->current_data.gathered.local_m = true;
        bridge->current_data.gathered.monotonic_m = true;
        bridge->current_data.gathered.terrain_m = true;
        bridge->current_data.gathered.bottom_clearance_m = true;
    }
    else if (strcmp(msg_type, "VFR_HUD") == 0) {
        mavlink_vfr_hud_t vfr_hud;
        mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
        
        bridge->current_data.altitude_m.relative = vfr_hud.alt;
        bridge->current_data.heading = DEG2RAD(vfr_hud.heading);
        bridge->current_data.throttle = vfr_hud.throttle;
    }
}

// Data thread function
void* serial_to_zmq_data_thread_func(void* arg) {
    serial_to_zmq_t* bridge = (serial_to_zmq_t*)arg;
    struct timespec ts;
    double out_time = 0.0;
    double print_time = 0.0;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double start_time = ts.tv_sec + ts.tv_nsec / 1e9;
    out_time = start_time + PUBLISH_DT;
    print_time = start_time + 1.0;
    
    while (bridge->running) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double current_time = ts.tv_sec + ts.tv_nsec / 1e9;
        
        // Process mavlink messages (non-blocking, will return immediately if no message)
        serial_to_zmq_listener_to_mavlink(bridge, false, 0.0, true, true);
        
        // Publish data at specified frequency
        if (current_time >= out_time) {
            out_time = current_time + PUBLISH_DT;
            
            pthread_mutex_lock(&bridge->data_lock);
            bridge->current_data.local_ts = current_time;
            
            // Serialize flight data
            char serialized_data[8192];
            size_t data_len = flight_data_serialize(&bridge->current_data, serialized_data, sizeof(serialized_data));
            
            if (data_len > 0) {
                zmq_publisher_send(bridge->pub_socket, TOPIC_MAVLINK_FLIGHT_DATA, serialized_data, data_len);
                bridge->current_data.message_count++;
            }
            pthread_mutex_unlock(&bridge->data_lock);
        }
        
        // Print status at lower frequency
        if (current_time >= print_time) {
            print_time = current_time + 1.0;
            pthread_mutex_lock(&bridge->data_lock);
            if (bridge->current_data.timestamp > 0) {
                printf("MAVLink timestamp: %.3f ms, local_ts: %.3f s\n", 
                       bridge->current_data.timestamp, bridge->current_data.local_ts);
            } else {
                printf("Waiting for MAVLink messages... (local_ts: %.3f s)\n", 
                       bridge->current_data.local_ts);
            }
            pthread_mutex_unlock(&bridge->data_lock);
        }
        
        usleep(100);  // 0.1ms
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

static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size,
                                   void* msg_dict, size_t msg_dict_size) {
    if (conn == NULL || msg_type == NULL || msg_dict == NULL) {
        return -1;
    }
    
    mavlink_connection_t* mconn = (mavlink_connection_t*)conn;
    if (mconn->fd < 0) {
        return -1;
    }
    
    mavlink_message_t msg;
    mavlink_status_t status;
    bool message_received = false;
    
    // Serial port: read byte-by-byte and parse character by character
    uint8_t byte;
    ssize_t bytes_read = serial_read_port(mconn->fd, &byte, 1);
    
    if (bytes_read <= 0) {
        if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            static int error_count = 0;
            if (error_count++ % 1000 == 0) {
                fprintf(stderr, "Error reading from serial port (every 1000 attempts): %s\n", strerror(errno));
            }
        }
        return -1;  // No data available or error
    }
    
    // Parse single byte
    uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status);
    
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
