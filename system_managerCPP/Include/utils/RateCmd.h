#ifndef RATE_CMD_H
#define RATE_CMD_H
#include "general.h"

#include <Eigen/Dense>

class Rate_Cmd {
public:
    Vector3d rpydot;

    Rate_Cmd(const Vector3d& rpydot = Vector3d::Zero());
};

#endif // RATE_CMD_H

