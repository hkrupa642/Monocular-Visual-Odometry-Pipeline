#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

class MotionEstimator {
public:
    MotionEstimator(double focalLength, const cv::Point2d& principalPoint);

    bool estimateMotion(
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        const std::vector<cv::DMatch>& matches,
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t
    );

private:
    double focalLength;
    cv::Point2d principalPoint;
};
