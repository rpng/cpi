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


#ifndef FEATUREINITIALIZER_H
#define FEATUREINITIALIZER_H

#include <mutex>
#include <thread>
#include <deque>
#include <unordered_map>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include "gtsam/JPLNavState.h"
#include "utils/quat_ops.h"
#include "utils/Config.h"
#include "utils/feature.h"


using namespace std;



using gtsam::symbol_shorthand::X; // X: our JPL states
using gtsam::symbol_shorthand::F; // F: our feature node


class FeatureInitializer {
public:

    /**
     * Default constructor
     * Will create the camera pose vector
     */
    FeatureInitializer(Config* config, gtsam::Values values, size_t nodect) {
        this->config = config;
        this->values = values;
        this->nodect = nodect;
        create_camera_poses();
    }


    /**
     * Function that takes in a feature and poses to initialize its 3D position
     */
    bool initialize_feature(feature& feat);



private:

    /// Global config object
    Config* config;

    /// Current number of nodes
    size_t nodect;

    /// Current best graph values
    gtsam::Values values;

    /// Vector of camera poses (q_GtoC, p_CinG)
    Eigen::VectorXd cam0poses;


    /// Will create the vector of camera poses
    void create_camera_poses();

    /// Will triangulate the feature using least squares optimization
    bool triangulate_feature(feature& feat);

    /// Will triangulate the feature using least squares optimization
    bool optimize_feature(feature& feat);

    /// Helper function that computes the error for a single gauss newton step
    double compute_stereo_error(std::vector<Eigen::Matrix<double,2,1>  > uv_vec,std::vector<int> cam_id, std::vector<int> cam_index, Eigen::MatrixXd rel_pose_vec, double alpha, double beta, double rho);


};



#endif /* FEATUREINITIALIZER_H */
