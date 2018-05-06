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


#pragma once
#include <string>
#include <sstream>
#include <iostream>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#pragma GCC diagnostic pop
#include <Eigen/Dense>


using namespace std;



/**
 * Returns a JPL quaternion from a rotation matrix
 */
inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::MatrixXd &rot) {

    assert(rot.cols() == 3);
    assert(rot.rows() == 3);
    Eigen::Matrix<double, 4, 1> q;
    double T = rot.trace();
    if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2))) {
        //cout << "case 1- " << endl;
        q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
        q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
        q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

    }
    else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
        //cout << "case 2- " << endl;
        q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
        q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
    }
    else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1))) {
        //cout << "case 3- " << endl;
        q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
        q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
        q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
    }
    else {
        //cout << "case 4- " << endl;
        q(3) = sqrt((1 + T) / 4);
        q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
        q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
        q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
    }
    if (q(3) < 0) {
        q = -q;
    }

    q = q / (q.norm());
    return q;
}


/**
 * Returns skew-symetric from a given 3x1 vector
 */
inline Eigen::MatrixXd skew_x(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    return w_x;
}


/**
 * Converts JPL quaterion to rotation matrix\
 */
inline Eigen::MatrixXd quat_2_Rot(Eigen::Matrix<double, 4, 1> q) {
    Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
    Eigen::MatrixXd Rot = (2 * pow(q(3, 0),2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                          2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
    return Rot;
}


/**
 * Multiply two JPL quaternions
 */
inline Eigen::Matrix<double, 4, 1> quat_multiply(Eigen::Matrix<double, 4, 1> q, Eigen::Matrix<double, 4, 1> p) {
    Eigen::Matrix<double, 4, 1> q_t;
    Eigen::Matrix<double, 4, 4> Qm;
    Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
    Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
    Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
    Qm(3, 3) = q(3, 0);
    q_t = Qm * p;
    if (q_t(3,0) <0){
        q_t*=-1;;
    }
    return q_t/q_t.norm();

}


/**
 * Returns vector portion of skew-symetric
 */
inline Eigen::Matrix<double, 3, 1> vee(Eigen::Matrix<double, 3, 3> w_x) {
    Eigen::Matrix<double, 3, 1> w;
    w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
    return w;
}



/**
 * SO3 Matrix expodential, returns rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> Exp(Eigen::Matrix<double, 3, 1> w) {

    Eigen::Matrix<double, 3, 3> w_x = skew_x(w);

    double theta = w.norm();

    Eigen::Matrix<double, 3, 3> R;

    if (theta ==0){
        R= Eigen::MatrixXd::Identity(3,3);
    }
    else{
        R= Eigen::MatrixXd::Identity(3, 3) + (sin(theta) / theta) * (w_x) + ((1 - cos(theta)) / pow(theta,2)) * w_x * w_x;
    }

    return R;

}



/**
 * SO3 Log function, returns a skew-symetric
 */
inline Eigen::Matrix<double, 3, 3> Log(Eigen::Matrix<double, 3, 3> R) {

    Eigen::Matrix<double, 3, 3> w_x;

    double theta = acos(.5 * (R.trace() - 1));

    w_x = (theta / (2 * sin(theta))) * (R - R.transpose());

    if (R != Eigen::MatrixXd::Identity(3,3)) {

        return w_x;
    }
    else{
        return Eigen::MatrixXd::Zero(3,3);
    }

}

/**
 * JPL Quaternion inverse
 */
inline Eigen::Matrix<double,4,1> Inv(Eigen::Matrix<double,4,1> q){
    Eigen::Matrix<double,4,1> qinv;

    qinv.block(0,0,3,1)= -q.block(0,0,3,1);
    qinv(3,0)= q(3,0);
    return qinv;

}


