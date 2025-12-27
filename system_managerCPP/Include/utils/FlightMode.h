#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

enum class FLIGHT_MODE {
    UNKNOWN = 0,
    OFFBOARD = 1,
    STABILIZED = 2,
    MANUAL = 3,
    ACRO = 4,
    RATTITUDE = 5,
    ALTCTL = 6,
    POSCTL = 7,
    LOITER = 8,
    MISSION = 9,
    RTL = 10,
    LAND = 11,
    RTGS = 12,
    TAKEOFF = 13,
    FOLLOWME = 14
};

#endif // FLIGHT_MODE_H

