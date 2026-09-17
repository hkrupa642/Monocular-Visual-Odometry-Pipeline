#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>

#include "FeatureDetector.h"
#include "MotionEstimator.h"
#include "Pose.h"

namespace fs = std::filesystem;

int main()
{
    std::cout << "=== Monocular Visual Odometry ===" << std::endl;

    std::vector<std::string> imagePaths;
    for (int i = 0; i < 10000; ++i) {
        std::string path = "data/" + std::to_string(i) + ".png";
        if (fs::exists(path)) {
            imagePaths.push_back(path);
        } else if (i > 0) {
            break;
        }
    }

    if (imagePaths.size() < 2) {
        std::cerr << "Error: Need at least 2 images in data/ directory." << std::endl;
        return -1;
    }

    std::cout << "Loaded " << imagePaths.size() << " images." << std::endl;

    cv::Mat firstImage = cv::imread(imagePaths[0], cv::IMREAD_GRAYSCALE);
    if (firstImage.empty()) {
        std::cerr << "Error: Could not read base image." << std::endl;
        return -1;
    }

    // Camera parameters (approximated from image resolution)
    double fx = firstImage.cols;
    double fy = firstImage.cols;
    double cx = firstImage.cols / 2.0;
    double cy = firstImage.rows / 2.0;

    FeatureDetector featureDetector;
    MotionEstimator motionEstimator(fx, cv::Point2d(cx, cy));
    Pose pose;

    std::ofstream trajectoryFile("trajectory.txt");
    if (!trajectoryFile.is_open()) {
        std::cerr << "Error: Failed to open trajectory.txt" << std::endl;
        return -1;
    }

    trajectoryFile << "# frame x y z\n";

    cv::Mat prevImg = firstImage;
    std::vector<cv::KeyPoint> prevKeypoints;
    cv::Mat prevDescriptors;

    featureDetector.detectAndCompute(prevImg, prevKeypoints, prevDescriptors);

    for (size_t frame = 1; frame < imagePaths.size(); ++frame) {
        cv::Mat currImg = cv::imread(imagePaths[frame], cv::IMREAD_GRAYSCALE);
        if (currImg.empty()) continue;

        std::vector<cv::KeyPoint> currKeypoints;
        cv::Mat currDescriptors;
        featureDetector.detectAndCompute(currImg, currKeypoints, currDescriptors);

        if (prevDescriptors.empty() || currDescriptors.empty()) {
            prevImg = currImg;
            prevKeypoints = currKeypoints;
            prevDescriptors = currDescriptors;
            continue;
        }

        auto goodMatches = featureDetector.matchDescriptors(prevDescriptors, currDescriptors);

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        bool success = motionEstimator.estimateMotion(
            prevKeypoints, currKeypoints, goodMatches, R, t
        );

        if (success) {
            pose.update(R, t);
            Eigen::Vector3d pos = pose.getPosition();

            trajectoryFile << frame << " " << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
            std::cout << "Frame " << frame << " | Position: [" 
                      << pos.x() << ", " << pos.y() << ", " << pos.z() << "]\n";
        } else {
            std::cerr << "Frame " << frame << ": Motion estimation failed.\n";
        }

        prevImg = currImg;
        prevKeypoints = currKeypoints;
        prevDescriptors = currDescriptors;
    }

    trajectoryFile.close();
    std::cout << "Done. Trajectory written to trajectory.txt" << std::endl;

    return 0;
}