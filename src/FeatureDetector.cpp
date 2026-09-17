#include "FeatureDetector.h"

FeatureDetector::FeatureDetector()
    : orb(cv::ORB::create(2000)),
      matcher(cv::NORM_HAMMING, false)
{
}

void FeatureDetector::detectAndCompute(
    const cv::Mat& image,
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors)
{
    orb->detectAndCompute(
        image,
        cv::noArray(),
        keypoints,
        descriptors
    );
}

std::vector<cv::DMatch> FeatureDetector::matchDescriptors(
    const cv::Mat& descriptors1,
    const cv::Mat& descriptors2)
{
    std::vector<std::vector<cv::DMatch>> knnMatches;

    matcher.knnMatch(
        descriptors1,
        descriptors2,
        knnMatches,
        2
    );

    std::vector<cv::DMatch> goodMatches;

    // Lowe ratio test
    const float ratio = 0.75f;

    for (const auto& matches : knnMatches) {
        if (matches.size() >= 2) {
            if (matches[0].distance < ratio * matches[1].distance) {
                goodMatches.push_back(matches[0]);
            }
        }
    }

    return goodMatches;
}
