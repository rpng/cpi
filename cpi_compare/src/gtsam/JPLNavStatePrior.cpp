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


#include "JPLNavStatePrior.h"


using namespace std;
using namespace gtsam;


/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector JPLNavStatePrior::evaluateError(const JPLNavState& state_i, boost::optional<Matrix&> H1) const {

    // Separate our variables from our state
    JPLQuaternion q_GtoK = state_i.q();
    Bias3 bg_K = state_i.bg();
    Velocity3 v_KinG = state_i.v();
    Bias3 ba_K = state_i.ba();
    Vector3 p_KinG = state_i.p();

    // Calculate quaternion error
    JPLQuaternion q_r = quat_multiply(q_GtoK, Inv(q_GtoI));

    //================================================================================
    //================================================================================
    //================================================================================

    // Our error vector [delta = (orientation, bg, v, ba, pos)]
    Vector15 error;

    // Error in our our state in respect to the measurement
    error.block(0,0,3,1) = 2*q_r.block(0,0,3,1);
    error.block(3,0,3,1) = bg_K - biasg;
    error.block(6,0,3,1) = v_KinG - v_IinG;
    error.block(9,0,3,1) = ba_K - biasa;
    error.block(12,0,3,1) = p_KinG - p_IinG;

    //================================================================================
    //================================================================================
    //================================================================================

    // Compute the Jacobian in respect to the first JPLNavState if needed
    if(H1) {

        // Derivative of our measurement in respect to the error state
        Eigen::Matrix<double,15,15> Hi = Eigen::Matrix<double,15,15>::Identity();

        // Derivative of our rotation in respect to the state
        Hi.block(0,0,3,3) = q_r(3,0)*Eigen::MatrixXd::Identity(3,3) + skew_x(q_r.block(0,0,3,1));

        // Save into the GTSAM jacobian
        *H1 = *OptionalJacobian<15,15>(Hi);

    }

    // Debug printing of error and Jacobians
    //if(H1) cout << endl << "JPLNavStatePrior H1" << endl << *H1 << endl << endl;
    //KeyFormatter keyFormatter = DefaultKeyFormatter;
    //cout << endl << "JPLNavStatePrior (" << keyFormatter(this->key()) << ")"  << endl << error.transpose() << endl;

    // Finally return our error vector!
    return error;

}