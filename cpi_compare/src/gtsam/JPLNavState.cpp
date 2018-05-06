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


#include "JPLNavState.h"


using namespace std;
using namespace gtsam;


/**
 * Main retract function, call on during optimization
 */
JPLNavState gtsam::JPLNavState::retract(const Vector15 &xi) const {

    // Calculate the update quaternion from the minimal representation
    Eigen::Matrix<double, 3, 1> dth = xi.block(0, 0, 3, 1);
    Eigen::Matrix<double, 3, 1> dq13 = ((std::sin(dth.norm() / 2) / dth.norm())) * dth;
    double dq4 = std::cos(dth.norm() / 2);

    // From the minimal representation, create the full 4x1 correction quaternion
    JPLQuaternion dq;
    dq << dq13, dq4;
    dq = dq / dq.norm();
    if (dq(3) < 0) {
        dq = -dq;
    }

    // Ensure that quaternion is valid
    if (std::isnan(dq.norm())) {
        dq << 0, 0, 0, 1.0;
    }

    // Update our current state values
    JPLQuaternion q = quat_multiply(dq,q_GtoI);
    Bias3 bg = biasg + xi.block(3, 0, 3, 1);
    Velocity3 v = v_IinG + xi.block(6, 0, 3, 1);
    Bias3 ba = biasa + xi.block(9, 0, 3, 1);
    Vector3 p = p_IinG + xi.block(12, 0, 3, 1);

    // Debug info
    //cout << endl << "===== JPLNavState =====" << endl;
    //JPLNavState state(q, bg, v, ba, p);
    //cout << state << endl;

    // Reconstruct and return this new state
    return JPLNavState(q, bg, v, ba, p);
}


/**
 * Local coordinates of our state
 * This should NOT be called....
 * This is basically a "box-minus" operation
 * Expanding about the current node's tangent space
 */
Vector15 gtsam::JPLNavState::localCoordinates(const JPLNavState &state) const {
    Vector15 localrep;
    localrep.block(0,0,3,1) = 2*quat_multiply(state.q(),Inv(q_GtoI)).block(0,0,3,1);
    localrep.block(3,0,3,1) = state.bg()-biasg;
    localrep.block(6,0,3,1) = state.v()-v_IinG;
    localrep.block(9,0,3,1) = state.ba()-biasa;
    localrep.block(12,0,3,1) = state.p()-p_IinG;
    return localrep;
}







