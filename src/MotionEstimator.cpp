#include "MotionEstimator.h"
#include <opencv2/calib3d.hpp>

MotionEstimator::MotionEstimator(double focalLength, const cv::Point2d& principalPoint)
    : focalLength(focalLength), principalPoint(principalPoint) {}

bool MotionEstimator::estimateMotion(
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::KeyPoint>& keypoints2,
    const std::vector<cv::DMatch>& matches,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t)
{
    if (matches.size() < 8) return false;

    std::vector<cv::Point2f> points1, points2;
    points1.reserve(matches.size());
    points2.reserve(matches.size());

    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(
        points1, points2,
        focalLength, principalPoint,
        cv::RANSAC, 0.999, 1.0, 1000,
        mask
    );

    if (E.empty()) return false;

    cv::Mat R_cv, t_cv;
    int inliers = cv::recoverPose(
        E, points1, points2, R_cv, t_cv,
        focalLength, principalPoint, mask
    );

    if (inliers < 5) return false;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R(r, c) = R_cv.at<double>(r, c);
        }
    }

    t(0) = t_cv.at<double>(0);
    t(1) = t_cv.at<double>(1);
    t(2) = t_cv.at<double>(2);

    return true;
}