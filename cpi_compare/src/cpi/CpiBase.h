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


#ifndef CPI_BASE_H
#define CPI_BASE_H


#include <Eigen/Dense>
#include "utils/quat_ops.h"


/**
 * Continuous Preintegration Theory for Graph-based Visual-Inertial Navigation
 * Authors: Kevin Eckenhoff, Patrick Geneva, and Guoquan Huang
 * http://udel.edu/~ghuang/papers/tr_cpi.pdf
 */
class CpiBase {

public:


    /**
     * Default constructor
     * @param sigma_w gyroscope white noise density (rad/s/sqrt(hz))
     * @param sigma_wb gyroscope random walk (rad/s^2/sqrt(hz))
     * @param sigma_a accelerometer white noise density (m/s^2/sqrt(hz))
     * @param sigma_ab accelerometer random walk (m/s^3/sqrt(hz))
     */
    CpiBase(double sigma_w, double sigma_wb, double sigma_a, double sigma_ab, bool imu_avg_= false) {
        // Calculate our covariance matrix
        Q_c.block(0,0,3,3) = std::pow(sigma_w,2)*eye3;
        Q_c.block(3,3,3,3) = std::pow(sigma_wb,2)*eye3;
        Q_c.block(6,6,3,3) = std::pow(sigma_a,2)*eye3;
        Q_c.block(9,9,3,3) = std::pow(sigma_ab,2)*eye3;
        imu_avg = imu_avg_;
        // Calculate our unit vectors, and their skews (used in bias jacobian calcs)
        e_1 << 1,0,0;
        e_2 << 0,1,0;
        e_3 << 0,0,1;
        e_1x = skew_x(e_1);
        e_2x = skew_x(e_2);
        e_3x = skew_x(e_3);
    }


    /**
     * This function sets the linearization points we are to preintegrate about
     * For model 2 we will also pass the q_GtoK and current gravity estimate
     */
    void setLinearizationPoints(Eigen::Matrix<double,3,1> b_w_lin_, Eigen::Matrix<double,3,1> b_a_lin_,
                                Eigen::Matrix<double,4,1> q_k_lin_ = Eigen::Matrix<double,4,1>::Zero(),
                                Eigen::Matrix<double,3,1> grav_ = Eigen::Matrix<double,3,1>::Zero()) {
        b_w_lin = b_w_lin_;
        b_a_lin = b_a_lin_;
        q_k_lin = q_k_lin_;
        grav = grav_;
    }


    /**
     * Function that handles new IMU messages, will precompound our means, jacobians, and measurement covariance
     */
    virtual void feed_IMU(double t_0, double t_1, Eigen::Matrix<double,3,1> w_m_0, Eigen::Matrix<double,3,1> a_m_0,
                          Eigen::Matrix<double,3,1> w_m_1 = Eigen::Matrix<double,3,1>::Zero(),
                          Eigen::Matrix<double,3,1> a_m_2 = Eigen::Matrix<double,3,1>::Zero()) = 0;



    // Flag if we should perform IMU averaging or not
    // For version 1 we should average the measurement
    // For version 2 we average the local true
    bool imu_avg = false;


    // Measurement Means
    double DT = 0;
    Eigen::Matrix<double,3,1> alpha_tau = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> beta_tau = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,4,1> q_k2tau;
    Eigen::Matrix<double,3,3> R_k2tau = Eigen::Matrix<double,3,3>::Identity();

    // Jacobians
    Eigen::Matrix<double,3,3> J_q = Eigen::Matrix<double,3,3>::Zero(); //ori jacob wrt b_w
    Eigen::Matrix<double,3,3> J_a = Eigen::Matrix<double,3,3>::Zero(); //alpha jacob wrt b_w
    Eigen::Matrix<double,3,3> J_b = Eigen::Matrix<double,3,3>::Zero(); //beta jacob wrt b_w
    Eigen::Matrix<double,3,3> H_a = Eigen::Matrix<double,3,3>::Zero(); //alpha jacob wrt b_a
    Eigen::Matrix<double,3,3> H_b = Eigen::Matrix<double,3,3>::Zero(); //beta jacob wrt b_a

    // Linearization points
    Eigen::Matrix<double,3,1> b_w_lin;
    Eigen::Matrix<double,3,1> b_a_lin;
    Eigen::Matrix<double,4,1> q_k_lin;

    //  Global gravity
    Eigen::Matrix<double,3,1> grav = Eigen::Matrix<double,3,1>::Zero();

    // Our continous-time measurement noise matrix
    Eigen::Matrix<double,12,12> Q_c = Eigen::Matrix<double,12,12>::Zero();

    // Our measurement covariance
    Eigen::Matrix<double,15,15> P_meas = Eigen::Matrix<double,15,15>::Zero();


    //==========================================================================
    // HELPER VARIABLES
    //==========================================================================

    // 3x3 identity matrix
    Eigen::Matrix<double,3,3> eye3 = Eigen::Matrix<double,3,3>::Identity();

    // Simple unit vectors (used in bias jacobian calculations)
    Eigen::Matrix<double,3,1> e_1;// = Eigen::Matrix<double,3,1>::Constant(1,0,0);
    Eigen::Matrix<double,3,1> e_2;// = Eigen::Matrix<double,3,1>::Constant(0,1,0);
    Eigen::Matrix<double,3,1> e_3;// = Eigen::Matrix<double,3,1>::Constant(0,0,1);

    // Calculate the skew-symetric of our unit vectors
    Eigen::Matrix<double,3,3> e_1x;// = skew_x(e_1);
    Eigen::Matrix<double,3,3> e_2x;// = skew_x(e_2);
    Eigen::Matrix<double,3,3> e_3x;// = skew_x(e_3);


};



#endif /* CPI_BASE_H */