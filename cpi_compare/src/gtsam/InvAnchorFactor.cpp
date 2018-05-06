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


#include "InvAnchorFactor.h"


using namespace std;
using namespace gtsam;

/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector InvAnchorFactor::evaluateError(const Point3& state_i, boost::optional<Matrix&> H1) const {


    // Separate our variables from our states
    Vector3 p_FinG_inv = state_i.vector();

    //================================================================================
    //================================================================================
    //================================================================================

    // Our error vector
    Vector2 error;

    // Extract the sub-components of our inverse depth
    double alpha = p_FinG_inv(0);
    double beta = p_FinG_inv(1);
    double rho = p_FinG_inv(2);

    // Here, we contruct just the [x/z y/z 1] feature
    Eigen::Matrix<double,3,1> a_b_1;
    a_b_1 << alpha, beta, 1.0;

    // Calculate the feature in the current camera frame
    Eigen::Matrix<double,3,3> R_canc_2_c = R_CtoI.transpose()*R_CAtoI;
    Eigen::Matrix<double,3,1> p_c_in_canc = p_IinCA - R_canc_2_c.transpose()*p_IinC;
    Eigen::Matrix<double,3,1> p_f_in_c = R_canc_2_c*(a_b_1 - rho*p_c_in_canc);

    // Our uv from inverse depth
    double u_hat = p_f_in_c(0,0)/p_f_in_c(2,0);
    double v_hat = p_f_in_c(1,0)/p_f_in_c(2,0);

    // Calculate the error
    error(0) = u_hat - uv(0);
    error(1) = v_hat - uv(1);
    //cout << "ERROR: " << endl << error.transpose() << endl;

    //================================================================================
    //================================================================================
    //================================================================================

    //Projection function jacobian
    Eigen::Matrix<double,2,3> H_proj;
    H_proj << (1.0)/p_f_in_c(2,0), 0.0, -p_f_in_c(0,0)/pow(p_f_in_c(2,0),2),
            0.0, (1.0)/p_f_in_c(2,0), -p_f_in_c(1,0)/pow(p_f_in_c(2,0),2);


    // Compute the Jacobian in respect to the feature if needed
    if (H1) {
        // UV in respect to the feature
        Eigen::Matrix<double,2,3> Hi = Eigen::Matrix<double,2,3>::Zero();
        Hi.block(0,0,2,2) = (H_proj*R_canc_2_c).block(0,0,2,2);
        Hi.block(0,2,2,1)= -H_proj*R_canc_2_c*p_c_in_canc;
        *H1 = *OptionalJacobian<2,3>(Hi);
        //cout << "H1 : " << endl << Hi << endl;
    }


    // Return our error
    return error;

}






