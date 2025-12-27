#ifndef RATE_CMD_H
#define RATE_CMD_H

#include <Eigen/Dense>

class Rate_Cmd {
public:
    Eigen::Vector3d rpydot;

    Rate_Cmd(const Eigen::Vector3d& rpydot = Eigen::Vector3d::Zero());
};

#endif // RATE_CMD_H

