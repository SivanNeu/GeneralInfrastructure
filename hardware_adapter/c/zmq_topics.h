#ifndef ZMQ_TOPICS_H
#define ZMQ_TOPICS_H

// Port definitions
#define TOPIC_MAVLINK_PORT 7790
#define TOPIC_GUIDANCE_CMD_PORT 7793

// Topic string definitions
#define TOPIC_MAVLINK_FLIGHT_DATA "FLIGHT_DATA"
#define TOPIC_GUIDANCE_CMD_ATTITUDE "quadAttitudeCmd"
#define TOPIC_GUIDANCE_CMD_VEL_NED "quadVelNedCmd"
#define TOPIC_GUIDANCE_CMD_VEL_BODY "quadVelBodyCmd"
#define TOPIC_GUIDANCE_CMD_ACC "quadAccCmd"
#define TOPIC_GUIDANCE_CMD_TAKEOFF "quadTakeoffCmd"
#define TOPIC_GUIDANCE_CMD_LAND "quadLandCmd"
#define TOPIC_GUIDANCE_CMD_ARM "quadArmCmd"

#endif // ZMQ_TOPICS_H

