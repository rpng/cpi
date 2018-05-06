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


#ifndef GTSAM_JPLNAVSTATE_H
#define GTSAM_JPLNAVSTATE_H


#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/quat_ops.h"

namespace gtsam {

    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Matrix<double,4,1> JPLQuaternion;

    /// Velocity is currently typedef'd to Vector3
    typedef Eigen::Vector3d Velocity3;

    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Vector3d Bias3;

    // Define a large 15 vector as this is the size of our correction vector
    typedef Eigen::Matrix<double,15,1> Vector15;


    /**
     * \brief JPL Navigation State
     * Contains orientation, position, velocity, and biases
     */
    class JPLNavState {
    private:

        JPLQuaternion q_GtoI; ///< Rotation from global to IMU
        Bias3 biasg; ///< Bias of the gyroscope
        Velocity3 v_IinG; ///< Velocity of IMU in global
        Bias3 biasa; ///< Bias of the accelerometer
        Vector3 p_IinG; ///< Position of IMU in global

    public:

        enum {
            dimension = 15
        };

        /// Default constructor
        JPLNavState() : q_GtoI(0,0,0,1), biasg(0,0,0), v_IinG(0,0,0), biasa(0,0,0), p_IinG(0,0,0) { }

        /// Construct from JPLNavState directly
        JPLNavState(const JPLNavState& navstate) {
            this->q_GtoI = navstate.q_GtoI;
            this->biasg = navstate.biasg;
            this->v_IinG = navstate.v_IinG;
            this->biasa = navstate.biasa;
            this->p_IinG = navstate.p_IinG;
        }

        /// Construct from orientation, position, velocity, and biases
        JPLNavState(const JPLQuaternion& q, const Bias3& bgi, const Velocity3& v, const Bias3& bai, const Vector3& p) :
                q_GtoI(q), biasg(bgi), v_IinG(v), biasa(bai), p_IinG(p) { }

        /// Return rotation quaternion.
        JPLQuaternion q() const {
            return q_GtoI;
        }

        /// Return position as Vector3
        Vector3 p() const {
            return p_IinG;
        }

        /// Return velocity as Vector3
        Vector3 v() const {
            return v_IinG;
        }

        /// Return ba as Vector3
        Vector3 ba() const {
            return biasa;
        }

        /// Return bg as Vector3
        Vector3 bg() const {
            return biasg;
        }

        /// Retract with optional derivatives (given correction, change this navstate)
        JPLNavState retract(const Vector15& xi) const;

        /// Converting function from our overparameterization to the local representation
        Vector15 localCoordinates(const JPLNavState& state) const;


        /// How this node gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const JPLNavState& state) {
            os << "q:[" << state.q()(0) << ", " << state.q()(1) << ", " << state.q()(2) << ", " << state.q()(3) << "]'" << endl;
            os << "bg:[" << state.bg()(0) << ", " << state.bg()(1) << ", " << state.bg()(2) << "]'" << endl;
            os << "v:[" << state.v()(0) << ", " << state.v()(1) << ", " << state.v()(2) << "]'" << endl;
            os << "ba:[" << state.ba()(0) << ", " << state.ba()(1) << ", " << state.ba()(2) << "]'" << endl;
            os << "p:[" << state.p()(0) << ", " << state.p()(1) << ", " << state.p()(2) << "]'" << endl;
            return os;
        }

        /// Print function for this node
        void print(const std::string& s = "") const {
            cout << s << *this << endl;
        }

        /// Equals function to compare this and another JPLNavState
        bool equals(const JPLNavState& other, double tol = 1e-8) const {
            return gtsam::equal(q_GtoI, other.q_GtoI, tol)
                   && gtsam::equal(biasg, other.biasg, tol)
                   && gtsam::equal(v_IinG, other.v_IinG, tol)
                   && gtsam::equal(biasa, other.biasa, tol)
                   && gtsam::equal(p_IinG, other.p_IinG, tol);
        }


    };

    template<>
    struct traits<JPLNavState> : internal::Manifold<JPLNavState> { };


} // namespace gtsam


#endif /* GTSAM_JPLNAVSTATE_H */