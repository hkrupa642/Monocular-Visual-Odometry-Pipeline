# Monocular Visual Odometry (VO) Pipeline

A lightweight C++ implementation of a frame-to-frame Monocular Visual Odometry pipeline. This project estimates camera movement (rotation and translation) from a single moving camera sequence using feature extraction, geometric constraints, and 3D pose integration.

---

## What This Project Does
* Reads sequential images from a monocular camera feed.
* Detects and tracks 2D visual features across consecutive frames.
* Computes the 5-DOF relative motion (rotation matrix $R$ and translation vector $t$ up to scale).
* Accumulates frame-to-frame motion to reconstruct a global camera trajectory saved to `trajectory.txt`.

---

## Project Structure & Architecture

### `FeatureDetector.h` / `FeatureDetector.cpp`
* **Purpose:** Handles feature detection and matching between consecutive images.
* **Key Functions:**
  * `detectAndCompute()`: Runs OpenCV's ORB detector to find keypoints and extract 256-bit binary descriptors.
  * `matchDescriptors()`: Uses `cv::BFMatcher` with Hamming distance to find matching features between frame pairs, filtering out weak matches using **Lowe's Ratio Test** (threshold set to 0.75).

### `MotionEstimator.h` / `MotionEstimator.cpp`
* **Purpose:** Calculates relative camera motion between two sets of matched 2D points.
* **Key Functions:**
  * `estimateMotion()`: Computes the Essential Matrix ($E$) using RANSAC to handle outlier matches, then decomposes $E$ via `cv::recoverPose()` to extract the relative rotation matrix $R \in SO(3)$ and normalized translation vector $t \in S^2$.

### `Pose.h` / `Pose.cpp`
* **Purpose:** Manages global state estimation.
* **Key Functions:**
  * `update()`: Converts $R$ and $t$ into a $4 \times 4$ homogeneous transformation matrix $T \in SE(3)$ and accumulates relative camera motion into the global coordinate frame: 
    $$T_{global} = T_{global} \cdot T_{relative}$$
  * `getPosition()`: Extracts the current 3D position vector $[x, y, z]^T$.

### `main.cpp`
* **Purpose:** Pipeline orchestration loop.
* **Workflow:** Loads image sequences, defines initial camera intrinsic parameters ($f_x, f_y, c_x, c_y$), loops through sequential frames, and writes calculated $x, y, z$ positions to disk.

---

## Computer Vision Concepts & Math Foundations

1. **Perception & Feature Filtering**
   * **ORB (Oriented FAST and Rotated BRIEF):** Provides fast, rotation-invariant feature point detection.
   * **Lowe's Ratio Test:** Compares the closest distance match to the second-closest match ($d_1 / d_2 < 0.75$) to reject ambiguous feature associations.
   * **RANSAC:** Random Sample Consensus eliminates remaining incorrect match pairs (outliers) during model fitting.

2. **Epipolar Geometry & Essential Matrix Estimation**
   * **Essential Matrix ($E$):** Encodes the geometric relationship between two normalized cameras for calibrated setups:
     $$E = [t]_\times R$$
   * **SVD Decomposition:** `cv::recoverPose()` uses Singular Value Decomposition on $E$ to extract valid $R$ and $t$ solutions while checking the positive depth constraint (cheirality check).

3. **State Estimation & Coordinate Frames**
   * **Scale Ambiguity:** Monocular VO can only recover translation vectors up to a relative unit scale ($\Vert{}t\Vert{} = 1$). Absolute physical distance requires additional sensors (IMU, stereo, depth) or ground-truth scaling.
   * **Pose Representation:** Motion uses Eigen linear algebra structures (`Eigen::Matrix3d`, `Eigen::Vector3d`, `Eigen::Matrix4d`) to perform rigid-body transformations in $SE(3)$ space.

---

## Build & Dependencies

* **C++ Compiler:** C++17 support
* **Build System:** CMake (v3.16+)
* **Dependencies:** OpenCV 4.x+, Eigen3

### Building the Project
```bash
cmake -B build
cmake --build build