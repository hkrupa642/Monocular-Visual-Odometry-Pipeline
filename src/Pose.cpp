#include "Pose.h"

Pose::Pose()
{
    pose = Eigen::Matrix4d::Identity();
}

void Pose::update(
    const Eigen::Matrix3d& relativeRotation,
    const Eigen::Vector3d& relativeTranslation)
{
    Eigen::Matrix4d relativePose = Eigen::Matrix4d::Identity();

    relativePose.block<3, 3>(0, 0) = relativeRotation;
    relativePose.block<3, 1>(0, 3) = relativeTranslation;

    pose = pose * relativePose;
}

const Eigen::Matrix4d& Pose::getPose() const
{
    return pose;
}

Eigen::Vector3d Pose::getPosition() const
{
    return pose.block<3, 1>(0, 3);
}
