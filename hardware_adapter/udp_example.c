// Simple example receiving and sending MAVLink v2 over UDP
// based on POSIX APIs (e.g. Linux, BSD, macOS).

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/time.h>

#include <mavlink/common/mavlink.h>


void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_message(const mavlink_message_t* message);
void handle_heartbeat(const mavlink_message_t* message);
void handle_attitude(const mavlink_message_t* message);
void handle_attitude_quaternion(const mavlink_message_t* message);
void handle_local_position_ned(const mavlink_message_t* message);
void handle_global_position_int(const mavlink_message_t* message);
void handle_highres_imu(const mavlink_message_t* message);
void handle_altitude(const mavlink_message_t* message);
void handle_vfr_hud(const mavlink_message_t* message);
const char* get_message_name(uint32_t msgid);

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);


int main(int argc, char* argv[])
{
    // Open UDP socket
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14540); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }

    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("setsockopt error: %s\n", strerror(errno));
        return -3;
    }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    while (true) {
        // For illustration purposes we don't bother with threads or async here
        // and just interleave receiving and sending.
        // This only works  if receive_some returns every now and then.
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return 0;
}

void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
        return;
    } else if (ret == 0) {
        // timeout, try again later
        return;
    }

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {
            // Print basic message info
            const char* msg_name = get_message_name(message.msgid);
            char src_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(src_addr->sin_addr), src_ip, INET_ADDRSTRLEN);
            printf("[%s:%d] Received %s (ID: %u) from system %u component %u\n",
                   src_ip, ntohs(src_addr->sin_port), msg_name, message.msgid, 
                   message.sysid, message.compid);
            
            // Handle specific message types
            handle_message(&message);
        }
    }
}

// Get message name from message ID
const char* get_message_name(uint32_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_ATTITUDE: return "ATTITUDE";
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: return "ATTITUDE_QUATERNION";
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: return "LOCAL_POSITION_NED";
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_HIGHRES_IMU: return "HIGHRES_IMU";
        case MAVLINK_MSG_ID_ALTITUDE: return "ALTITUDE";
        case MAVLINK_MSG_ID_VFR_HUD: return "VFR_HUD";
        case MAVLINK_MSG_ID_ATTITUDE_TARGET: return "ATTITUDE_TARGET";
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: return "POSITION_TARGET_LOCAL_NED";
        case MAVLINK_MSG_ID_COMMAND_ACK: return "COMMAND_ACK";
        case MAVLINK_MSG_ID_DISTANCE_SENSOR: return "DISTANCE_SENSOR";
        case MAVLINK_MSG_ID_OPTICAL_FLOW: return "OPTICAL_FLOW";
        default: return "UNKNOWN";
    }
}

// Route message to appropriate handler
void handle_message(const mavlink_message_t* message) {
    switch (message->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_heartbeat(message);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            handle_attitude(message);
            break;
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            handle_attitude_quaternion(message);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            handle_local_position_ned(message);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            handle_global_position_int(message);
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            handle_highres_imu(message);
            break;
        case MAVLINK_MSG_ID_ALTITUDE:
            handle_altitude(message);
            break;
        case MAVLINK_MSG_ID_VFR_HUD:
            handle_vfr_hud(message);
            break;
        default:
            printf("  (No specific handler for this message type)\n");
            break;
    }
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("  HEARTBEAT: type=%u, autopilot=", heartbeat.type);
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("GENERIC");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ARDUPILOT");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("OTHER(%u)", heartbeat.autopilot);
            break;
    }
    printf(", base_mode=0x%02x, custom_mode=%u, system_status=%u, mavlink_version=%u\n",
           heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status, heartbeat.mavlink_version);
}

void handle_attitude(const mavlink_message_t* message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(message, &attitude);
    
    printf("  ATTITUDE: roll=%.3f, pitch=%.3f, yaw=%.3f (rad)\n",
           attitude.roll, attitude.pitch, attitude.yaw);
    printf("           roll_rate=%.3f, pitch_rate=%.3f, yaw_rate=%.3f (rad/s)\n",
           attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed);
    printf("           time_boot_ms=%u\n", attitude.time_boot_ms);
}

void handle_attitude_quaternion(const mavlink_message_t* message)
{
    mavlink_attitude_quaternion_t att_quat;
    mavlink_msg_attitude_quaternion_decode(message, &att_quat);
    
    printf("  ATTITUDE_QUATERNION: q=[%.4f, %.4f, %.4f, %.4f] (w, x, y, z)\n",
           att_quat.q1, att_quat.q2, att_quat.q3, att_quat.q4);
    printf("                       roll_rate=%.3f, pitch_rate=%.3f, yaw_rate=%.3f (rad/s)\n",
           att_quat.rollspeed, att_quat.pitchspeed, att_quat.yawspeed);
    printf("                       time_boot_ms=%u\n", att_quat.time_boot_ms);
}

void handle_local_position_ned(const mavlink_message_t* message)
{
    mavlink_local_position_ned_t local_pos;
    mavlink_msg_local_position_ned_decode(message, &local_pos);
    
    printf("  LOCAL_POSITION_NED: pos=[%.2f, %.2f, %.2f] (m, NED)\n",
           local_pos.x, local_pos.y, local_pos.z);
    printf("                      vel=[%.2f, %.2f, %.2f] (m/s, NED)\n",
           local_pos.vx, local_pos.vy, local_pos.vz);
    printf("                      time_boot_ms=%u\n", local_pos.time_boot_ms);
}

void handle_global_position_int(const mavlink_message_t* message)
{
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(message, &global_pos);
    
    printf("  GLOBAL_POSITION_INT: lat=%.7f, lon=%.7f, alt=%.2f (m)\n",
           global_pos.lat / 1e7, global_pos.lon / 1e7, global_pos.alt / 1e3);
    printf("                        relative_alt=%.2f (m)\n", global_pos.relative_alt / 1e3);
    printf("                        vel=[%.2f, %.2f, %.2f] (m/s, NED)\n",
           global_pos.vx / 100.0, global_pos.vy / 100.0, global_pos.vz / 100.0);
    printf("                        hdg=%.1f (deg), time_boot_ms=%u\n",
           global_pos.hdg / 100.0, global_pos.time_boot_ms);
}

void handle_highres_imu(const mavlink_message_t* message)
{
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(message, &imu);
    
    printf("  HIGHRES_IMU: accel=[%.3f, %.3f, %.3f] (m/s²)\n",
           imu.xacc, imu.yacc, imu.zacc);
    printf("               gyro=[%.3f, %.3f, %.3f] (rad/s)\n",
           imu.xgyro, imu.ygyro, imu.zgyro);
    printf("               mag=[%.3f, %.3f, %.3f] (gauss)\n",
           imu.xmag, imu.ymag, imu.zmag);
    printf("               abs_pressure=%.2f (hPa), diff_pressure=%.2f (hPa)\n",
           imu.abs_pressure, imu.diff_pressure);
    printf("               pressure_alt=%.2f (m), temperature=%.2f (°C)\n",
           imu.pressure_alt, imu.temperature);
    printf("               time_usec=%lu\n", (unsigned long)imu.time_usec);
}

void handle_altitude(const mavlink_message_t* message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(message, &altitude);
    
    printf("  ALTITUDE: altitude_monotonic=%.2f, altitude_amsl=%.2f, altitude_local=%.2f (m)\n",
           altitude.altitude_monotonic, altitude.altitude_amsl, altitude.altitude_local);
    printf("            altitude_relative=%.2f, altitude_terrain=%.2f, bottom_clearance=%.2f (m)\n",
           altitude.altitude_relative, altitude.altitude_terrain, altitude.bottom_clearance);
    printf("            time_usec=%lu\n", (unsigned long)altitude.time_usec);
}

void handle_vfr_hud(const mavlink_message_t* message)
{
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode(message, &vfr_hud);
    
    printf("  VFR_HUD: airspeed=%.2f (m/s), groundspeed=%.2f (m/s)\n",
           vfr_hud.airspeed, vfr_hud.groundspeed);
    printf("           heading=%.1f (deg), throttle=%.1f%%\n",
           vfr_hud.heading, vfr_hud.throttle);
    printf("           alt=%.2f (m), climb=%.2f (m/s)\n",
           vfr_hud.alt, vfr_hud.climb);
}

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        send_heartbeat(socket_fd, src_addr, src_addr_len);
        last_time = current_time;
    }
}

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}
