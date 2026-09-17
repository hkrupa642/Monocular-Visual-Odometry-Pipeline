#pragma once

#include <Eigen/Dense>

class Pose {
public:
    Pose();

    void update(
        const Eigen::Matrix3d& relativeRotation,
        const Eigen::Vector3d& relativeTranslation
    );

    const Eigen::Matrix4d& getPose() const;

    Eigen::Vector3d getPosition() const;

private:
    Eigen::Matrix4d pose;
};
