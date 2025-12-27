#ifndef PIXEL_H
#define PIXEL_H

#include <Eigen/Dense>

class Pixel {
public:
    Eigen::Vector2d px;

    Pixel(const Eigen::Vector2d& px = Eigen::Vector2d::Zero());
};

#endif // PIXEL_H

