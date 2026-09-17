#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureDetector {
public:
    FeatureDetector();

    void detectAndCompute(
        const cv::Mat& image,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors
    );

    std::vector<cv::DMatch> matchDescriptors(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2
    );

private:
    cv::Ptr<cv::ORB> orb;
    cv::BFMatcher matcher;
};
