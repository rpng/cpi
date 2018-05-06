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


#include "ImuFactorCPIv2.h"


using namespace std;
using namespace gtsam;



/**
 * Called on when optimizing to get the error of this measurement
 */
gtsam::Vector ImuFactorCPIv2::evaluateError(const JPLNavState &state_i, const JPLNavState &state_j,
                                          boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

    // Separate our variables from our states
    JPLQuaternion q_GtoK = state_i.q();
    JPLQuaternion q_GtoK1 = state_j.q();
    Bias3 bg_K = state_i.bg();
    Bias3 bg_K1 = state_j.bg();
    Velocity3 v_KinG = state_i.v();
    Velocity3 v_K1inG = state_j.v();
    Bias3 ba_K = state_i.ba();
    Bias3 ba_K1 = state_j.ba();
    Vector3 p_KinG = state_i.p();
    Vector3 p_K1inG = state_j.p();

    //================================================================================
    //================================================================================
    //================================================================================

    // Calculate effect of bias on the orientation
    Eigen::Matrix<double, 3, 3> ExpB = Exp(-J_q.block(0, 0, 3, 3)*( bg_K - bg_lin ));
    JPLQuaternion q_b = rot_2_quat(ExpB);

    // Quaternions used in Jacobian calculations
    JPLQuaternion q_n = quat_multiply(q_GtoK1, Inv(q_GtoK));
    JPLQuaternion q_rminus = quat_multiply(q_n, Inv(q_KtoK1));
    JPLQuaternion q_r = quat_multiply(q_rminus, q_b);
    JPLQuaternion q_m = quat_multiply(Inv(q_b), q_KtoK1);

    // q_k update delta
    JPLQuaternion q_kR= quat_multiply(q_GtoK, Inv(q_K_lin));
    Eigen::Matrix<double,3,1> dthk = 2*q_kR.block(0,0,3,1);

    // Calculate the expected measurement values from the state
    Vector3 alphahat = quat_2_Rot(q_GtoK)*(p_K1inG - p_KinG - v_KinG*deltatime);
    alphahat = alphahat - J_alpha*(bg_K - bg_lin) - H_alpha*(ba_K - ba_lin)- O_alpha*dthk;
    Vector3 betahat = quat_2_Rot(q_GtoK)*(v_K1inG - v_KinG);
    betahat = betahat - J_beta*(bg_K - bg_lin) - H_beta*(ba_K - ba_lin)- O_beta*dthk;

    //================================================================================
    //================================================================================
    //================================================================================


    // Our error vector [delta = (orientation, bg, v, ba, pos)]
    Vector15 error;

    // Error in our our state in respect to the measurement
    error.block(0,0,3,1) = 2*q_r.block(0,0,3,1);
    error.block(3,0,3,1) = bg_K1 - bg_K;
    error.block(6,0,3,1) = betahat - beta;
    error.block(9,0,3,1) = ba_K1 - ba_K;
    error.block(12,0,3,1) = alphahat - alpha;


    //================================================================================
    //================================================================================
    //================================================================================



    // Compute the Jacobian in respect to the first JPLNavState if needed
    if(H1) {

        // Create our five jacobians of the measurement in respect to the state
        Eigen::Matrix<double,15,3> H_theta = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_biasg = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_velocity = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_biasa = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_position = Eigen::Matrix<double,15,3>::Zero();

        //===========================================================
        // Derivative of q_meas in respect to state theta (t=K)
        H_theta.block(0,0,3,3) = -((q_n(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q_n.block(0, 0, 3, 1))) *
                                    (q_m(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q_m.block(0, 0, 3, 1)))
                                      + q_n.block(0, 0, 3, 1) * (q_m.block(0, 0, 3, 1)).transpose());
        // Derivative of beta in respect to state theta (t=K)
        H_theta.block(6,0,3,3) = skew_x(quat_2_Rot(q_GtoK)*(v_K1inG - v_KinG))-
                O_beta*(q_kR(3,0)*Eigen::MatrixXd::Identity(3, 3)+ skew_x(q_kR.block(0,0,3,1)));
        // Derivative of alpha in respect to state theta (t=K)
        H_theta.block(12,0,3,3) = skew_x(quat_2_Rot(q_GtoK)*(p_K1inG - p_KinG - v_KinG*deltatime))-
                O_alpha*(q_kR(3,0)*Eigen::MatrixXd::Identity(3, 3)+ skew_x(q_kR.block(0,0,3,1)));

        //===========================================================
        // Derivative of q_meas in respect to state biasg (t=K)
        H_biasg.block(0,0,3,3) = (q_rminus(3, 0)*Eigen::MatrixXd::Identity(3, 3) - skew_x(q_rminus.block(0, 0, 3, 1)))*J_q.block(0, 0, 3, 3);
        // Derivative of biasg in respect to state biasg (t=K)
        H_biasg.block(3,0,3,3) = -Eigen::MatrixXd::Identity(3, 3);
        // Derivative of beta in respect to state biasg (t=K)
        H_biasg.block(6,0,3,3) = -J_beta;
        // Derivative of alpha in respect to state biasg (t=K)
        H_biasg.block(12,0,3,3) = -J_alpha;

        //===========================================================
        // Derivative of beta in respect to state velocity (t=K)
        H_velocity.block(6,0,3,3) = -quat_2_Rot(q_GtoK);
        // Derivative of alpha in respect to state velocity (t=K)
        H_velocity.block(12,0,3,3) = -deltatime*quat_2_Rot(q_GtoK);

        //===========================================================
        // Derivative of beta in respect to state biasa (t=K)
        H_biasa.block(6,0,3,3) = -H_beta;
        // Derivative of biasa in respect to state biasa (t=K)
        H_biasa.block(9,0,3,3) = -Eigen::MatrixXd::Identity(3, 3);
        // Derivative of alpha in respect to state biasa (t=K)
        H_biasa.block(12,0,3,3) = -H_alpha;

        //===========================================================
        // Derivative of alpha in respect to state position (t=K)
        H_position.block(12,0,3,3) = -quat_2_Rot(q_GtoK);

        //===========================================================
        // Reconstruct the whole Jacobian from the different columns
        Eigen::Matrix<double,15,15> Hi;
        Hi.block(0,0,15,3) = H_theta;
        Hi.block(0,3,15,3) = H_biasg;
        Hi.block(0,6,15,3) = H_velocity;
        Hi.block(0,9,15,3) = H_biasa;
        Hi.block(0,12,15,3) = H_position;
        *H1 = *OptionalJacobian<15,15>(Hi);
    }


    // Compute the Jacobian in respect to the second JPLNavState if needed
    if(H2) {

        // Create our five jacobians of the measurement in respect to the state
        Eigen::Matrix<double,15,3> H_theta = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_biasg = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_velocity = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_biasa = Eigen::Matrix<double,15,3>::Zero();
        Eigen::Matrix<double,15,3> H_position = Eigen::Matrix<double,15,3>::Zero();

        //===========================================================
        // Derivative of q_meas in respect to state theta (t=K+1)
        H_theta.block(0,0,3,3) = q_r(3, 0)*Eigen::MatrixXd::Identity(3, 3) + skew_x(q_r.block(0, 0, 3, 1));

        //===========================================================
        // Derivative of biasg in respect to state biasg (t=K+1)
        H_biasg.block(3,0,3,3) = Eigen::MatrixXd::Identity(3, 3);

        //===========================================================
        // Derivative of beta in respect to state velocity (t=K+1)
        H_velocity.block(6,0,3,3) = quat_2_Rot(q_GtoK);

        //===========================================================
        // Derivative of biasa in respect to state biasa (t=K+1)
        H_biasa.block(9,0,3,3) = Eigen::MatrixXd::Identity(3, 3);

        //===========================================================
        // Derivative of alpha in respect to state position (t=K+1)
        H_position.block(12,0,3,3) = quat_2_Rot(q_GtoK);


        //===========================================================
        // Reconstruct the whole Jacobian
        Eigen::Matrix<double,15,15> Hj;
        Hj.block(0,0,15,3) = H_theta;
        Hj.block(0,3,15,3) = H_biasg;
        Hj.block(0,6,15,3) = H_velocity;
        Hj.block(0,9,15,3) = H_biasa;
        Hj.block(0,12,15,3) = H_position;
        *H2 = *OptionalJacobian<15,15>(Hj);

    }

    // Debug printing of error and Jacobians
    //if(H1) cout << endl << "ImuFactorCPIv1 H1" << endl << *H1 << endl << endl;
    //if(H2) cout << endl << "ImuFactorCPIv1 H2" << endl << *H2 << endl << endl;
    //KeyFormatter keyFormatter = DefaultKeyFormatter;
    //cout << endl << "ImuFactorCPIv1 (" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ")" << endl << error.transpose() << endl;

    // Finally return our error vector!
    return error;
}






