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


#include "JPLImageUVFactor.h"


using namespace std;
using namespace gtsam;

/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector JPLImageUVFactor::evaluateError(const JPLNavState& state_i, const Point3& state_j,
                                              boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

    // Separate our variables from our states
    JPLQuaternion q_GtoI = state_i.q();
    Vector3 p_IinG = state_i.p();
    Vector3 p_FinG = state_j.vector();

    //================================================================================
    //================================================================================
    //================================================================================

    // Our error vector
    Vector2 error;

    // Project onto the image plane
    Eigen::Matrix<double,3,1> p_f_in_i = quat_2_Rot(q_GtoI)*(p_FinG - p_IinG);
    Eigen::Matrix<double,3,1> p_f_in_c = R_CtoI.transpose()*p_f_in_i + p_IinC;
    double u_hat = p_f_in_c(0,0)/p_f_in_c(2,0);
    double v_hat = p_f_in_c(1,0)/p_f_in_c(2,0);

    // Calculate the error
    error(0) = u_hat - uv(0);
    error(1) = v_hat - uv(1);
    //cout << "ERROR : " << endl << error.transpose() << endl;

    //================================================================================
    //================================================================================
    //================================================================================

    //Projection function jacobian
    Eigen::Matrix<double,2,3> H_proj;
    H_proj << 1.0/p_f_in_c(2,0), 0.0, -p_f_in_c(0,0)/pow(p_f_in_c(2,0),2),
            0.0, 1.0/p_f_in_c(2,0), -p_f_in_c(1,0)/pow(p_f_in_c(2,0),2);

    // Compute the Jacobian in respect to the JPLNavState if needed
    if (H1) {
        Eigen::Matrix<double,2,15> Hi = Eigen::Matrix<double,2,15>::Zero();
        // UV in respect to the theta error state
        Hi.block(0,0,2,3) = H_proj*R_CtoI.transpose()*skew_x(p_f_in_i);
        // UV in respect to the position
        Hi.block(0,12,2,3) = -H_proj*R_CtoI.transpose()*quat_2_Rot(q_GtoI);
        *H1 = *OptionalJacobian<2,15>(Hi);
        //cout << "H1 : " << endl << Hi.block(0,0,2,3) << endl << Hi.block(0,12,2,3) << endl;
    }


    // Compute the Jacobian in respect to the feature if needed
    if (H2) {
        // UV in respect to the feature
        Eigen::Matrix<double,2,3> Hi = Eigen::Matrix<double,2,3>::Zero();
        Hi = H_proj*R_CtoI.transpose()*quat_2_Rot(q_GtoI);
        *H2 = *OptionalJacobian<2,3>(Hi);
        //cout << "H2 : " << endl << Hi << endl;
    }


    // Return our error
    return error;

}






