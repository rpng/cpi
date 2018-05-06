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


#ifndef GTSAM_JPLNAVSTATEPRIOR_H
#define GTSAM_JPLNAVSTATEPRIOR_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "JPLNavState.h"
#include "utils/quat_ops.h"

using namespace std;
using namespace gtsam;


namespace gtsam {

    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Matrix<double, 4, 1> JPLQuaternion;

    /// Velocity is currently typedef'd to Vector3
    typedef Eigen::Vector3d Velocity3;

    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Vector3d Bias3;

    // Define a large 15 vector as this is the size of our correction vector
    typedef Eigen::Matrix<double, 15, 1> Vector15;


    /**
     * \brief JPL Navigation State Prior
     * Our initial state at initialization
     */
    class JPLNavStatePrior : public NoiseModelFactor1<JPLNavState> {
    private:

        JPLQuaternion q_GtoI; ///< Rotation from global to IMU
        Bias3 biasg; ///< Bias of the gyroscope
        Velocity3 v_IinG; ///< Velocity of IMU in global
        Bias3 biasa; ///< Bias of the accelerometer
        Vector3 p_IinG; ///< Position of IMU in global

    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        JPLNavStatePrior(Key state_i, Eigen::Matrix<double, 15, 15> covariance, const JPLQuaternion &q,
                         const Bias3 &bgi,
                         const Velocity3 &v, const Bias3 &bai, const Vector3 &p) :
                NoiseModelFactor1<JPLNavState>(noiseModel::Gaussian::Covariance(covariance), state_i) {
            // Measurement
            this->q_GtoI = q;
            this->biasg = bgi;
            this->v_IinG = v;
            this->biasa = bai;
            this->p_IinG = p;
        }

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

        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState &state_i, boost::optional<Matrix &> H1 = boost::none) const;


        /// How this node gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const JPLNavStatePrior &state) {
            os << "q:[" << state.q()(0) << ", " << state.q()(1) << ", " << state.q()(2) << ", " << state.q()(3) << "]'" << endl;
            os << "bg:[" << state.bg()(0) << ", " << state.bg()(1) << ", " << state.bg()(2) << "]'" << endl;
            os << "v:[" << state.v()(0) << ", " << state.v()(1) << ", " << state.v()(2) << "]'" << endl;
            os << "ba:[" << state.ba()(0) << ", " << state.ba()(1) << ", " << state.ba()(2) << "]'" << endl;
            os << "p:[" << state.p()(0) << ", " << state.p()(1) << ", " << state.p()(2) << "]'" << endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string &s, const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "JPLNavStatePrior(" << keyFormatter(this->key()) << ")" << std::endl;
            std::cout << "  measured: " << std::endl << *this << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const JPLNavStatePrior *e = dynamic_cast<const JPLNavStatePrior *>(&expected);
            if (e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor1<JPLNavState>::equals(*e, tol)
                   && gtsam::equal(q_GtoI, e->q_GtoI, tol)
                   && gtsam::equal(biasg, e->biasg, tol)
                   && gtsam::equal(v_IinG, e->v_IinG, tol)
                   && gtsam::equal(biasa, e->biasa, tol)
                   && gtsam::equal(p_IinG, e->p_IinG, tol);
        }


    };

} // namespace gtsam



#endif /* GTSAM_JPLNAVSTATEPRIOR_H */