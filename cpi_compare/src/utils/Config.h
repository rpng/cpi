/**
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>

class Config {

public:

    /// Default constructor
    Config(){}

    /// If we should use serial simulation
    bool useSerialSIM;

    /// Path to simulation dataset
    std::string simPath;

    /// If we should use an inverse depth parameterization for our features
    bool useInverseDepth;

    /// Amount of "lag" we should have in our smoother
    int lagSmootherAmount;

    /// Min number of poses to initalize a feature
    int minPoseFeatureInit;

    /// Amount of poses that we should stop updating the feature after
    int uvWindowSize;

    /// If we should use the groundtruth to initialize
    bool useGroundTruthInit;

    // Rates
    int imuRate;
    int camRate;

    /// Amount of IMU we should wait to initialize to
    int imuWait;

    /// Relative transform between CAM0 and IMU
    Eigen::Matrix<double,3,1> p_IinC0;
    Eigen::Matrix<double,3,3> R_C0toI;

    /// Relative transform between CAM1 and IMU
    Eigen::Matrix<double,3,1> p_IinC1;
    Eigen::Matrix<double,3,3> R_C1toI;

    /// Assumed "true" global gravity
    Eigen::Matrix<double,3,1> gravity;

    /// Noise values for the image
    double sigma_camera; ///< Noise value for CAMERA points
    double sigma_camera_sq; ///< Noise value for CAMERA points squared

    // Noise values for the IMU
    double sigma_g; ///< Gyro white noise
    double sigma_g_sq; ///< Gyro white noise squared
    double sigma_wg; ///< Gyro bias walk
    double sigma_wg_sq; ///< Gyro bias walk squared
    double sigma_a; ///< Acceleration white noise
    double sigma_a_sq; ///< Acceleration white noise squared
    double sigma_wa; ///< Acceleration bias walk
    double sigma_wa_sq; ///< Acceleration bias walk squared

};




#endif /* CONFIG_H */