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


#ifndef GTSAM_IMUFACTORCPIv1_H
#define GTSAM_IMUFACTORCPIv1_H

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "JPLNavState.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {


    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Matrix<double,4,1> JPLQuaternion;

    /// Bias for a sensor is currently typedef'd to Vector3
    typedef Eigen::Vector3d Bias3;

    // Define a large 15 vector as this is the size of our error state
    typedef Eigen::Matrix<double,15,1> Vector15;


    /**
     * \brief Continuous Preintegration Factor Model 1
     * Links two full JPL Navigation States with an IMU preintegrated measurement
     */
    class ImuFactorCPIv1 : public NoiseModelFactor2<JPLNavState, JPLNavState> {
    private:

        Vector3 alpha; ///< preintegration measurement due to position
        Vector3 beta; ///< preintegration measurement due to velocity
        JPLQuaternion q_KtoK1; ///< preintegration measurement due to rotation

        Bias3 ba_lin; ///< original acceleration bias linerization point
        Bias3 bg_lin; ///< original gyroscope bias linearization point

        Eigen::Matrix<double,3,3> J_q; ///< jacobian of the preintegrated rotation in respect to the gyro bias correction
        Eigen::Matrix<double,3,3> J_beta; ///<  jacobian of the preintegrated velocity in respect to the gyro bias correction
        Eigen::Matrix<double,3,3> J_alpha; ///<  jacobian of the preintegrated position in respect to the gyro bias correction

        Eigen::Matrix<double,3,3> H_beta; ///<  jacobian of the preintegrated velocity in respect to the accelerometer bias correction
        Eigen::Matrix<double,3,3> H_alpha; ///<  jacobian of the preintegrated position in respect to the accelerometer bias correction

        double deltatime; ///< time in seconds that this measurement is over
        Vector3 grav; ///< global gravity (should be the same for all measurements)

    public:

        /// Construct from the two linking JPLNavStates, preingration measurement, and its covariance
        ImuFactorCPIv1(Key state_i, Key state_j, Eigen::Matrix<double,15,15> covariance, double deltatime,
                      Vector3 grav, Vector3 alpha, Vector3 beta, JPLQuaternion q_KtoK1, Bias3 ba_lin, Bias3 bg_lin,
                      Eigen::Matrix<double,3,3> J_q, Eigen::Matrix<double,3,3> J_beta, Eigen::Matrix<double,3,3> J_alpha,
                      Eigen::Matrix<double,3,3> H_beta, Eigen::Matrix<double,3,3> H_alpha) :
                NoiseModelFactor2<JPLNavState, JPLNavState>(noiseModel::Gaussian::Covariance(covariance), state_i, state_j) {

            // Measurement
            this->alpha = alpha;
            this->beta = beta;
            this->q_KtoK1 = q_KtoK1;
            this->ba_lin = ba_lin;
            this->bg_lin = bg_lin;
            // Precomputed Jacobians
            this->J_q = J_q;
            this->J_beta = J_beta;
            this->J_alpha = J_alpha;
            this->H_beta = H_beta;
            this->H_alpha = H_alpha;
            // Static values
            this->deltatime = deltatime;
            this->grav = grav;

        }

        /// Return alpha measurement.
        double dt() const {
            return deltatime;
        }

        /// Return alpha measurement.
        Vector3 m_alpha() const {
            return alpha;
        }

        /// Return beta measurement.
        Vector3 m_beta() const {
            return beta;
        }

        /// Return rotation delta measurement.
        JPLQuaternion m_q() const {
            return q_KtoK1;
        }

        /// Return acceleration bias linearization point.
        Bias3 m_balin() const {
            return ba_lin;
        }

        /// Return acceleration bias linearization point.
        Bias3 m_bglin() const {
            return bg_lin;
        }

        /// Returns global gravity
        Bias3 gravity() const {
            return grav;
        }


        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const JPLNavState& state_i, const JPLNavState& state_j,
                                    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const ImuFactorCPIv1& factor) {
            os << "dt:[" << factor.dt() << "]'" << endl;
            os << "alpha:[" << factor.m_alpha()(0) << ", " << factor.m_alpha()(1) << ", " << factor.m_alpha()(2) << "]'" << endl;
            os << "beta:[" << factor.m_beta()(0) << ", " << factor.m_beta()(1) << ", " << factor.m_beta()(2) << "]'" << endl;
            os << "dq_KtoK1:[" << factor.m_q()(0) << ", " << factor.m_q()(1) << ", " << factor.m_q()(2) << ", " << factor.m_q()(3) << "]'" << endl;
            os << "ba_lin:[" << factor.m_balin()(0) << ", " << factor.m_balin()(1) << ", " << factor.m_balin()(2) << "]'" << endl;
            os << "bg_lin:[" << factor.m_bglin()(0) << ", " << factor.m_bglin()(1) << ", " << factor.m_bglin()(2) << "]'" << endl;
            os << "gravity:[" << factor.gravity()(0) << ", " << factor.gravity()(1) << ", " << factor.gravity()(2) << "]'" << endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "ImuFactorCPIv1(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2()) << ")" << std::endl;
            std::cout << "  measured: " << std::endl << *this << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const ImuFactorCPIv1 *e =  dynamic_cast<const ImuFactorCPIv1*>(&expected);
            if(e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor2<JPLNavState,JPLNavState>::equals(*e, tol)
                   && gtsam::equal(deltatime, e->deltatime, tol)
                   && gtsam::equal(alpha, e->alpha, tol)
                   && gtsam::equal(beta, e->beta, tol)
                   && gtsam::equal(q_KtoK1, e->q_KtoK1, tol)
                   && gtsam::equal(ba_lin, e->ba_lin, tol)
                   && gtsam::equal(bg_lin, e->bg_lin, tol)
                   && gtsam::equal(J_q, e->J_q, tol)
                   && gtsam::equal(J_beta, e->J_beta, tol)
                   && gtsam::equal(J_alpha, e->J_alpha, tol)
                   && gtsam::equal(H_beta, e->H_beta, tol)
                   && gtsam::equal(H_alpha, e->H_alpha, tol)
                   && gtsam::equal(grav, e->grav, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_IMUFACTORCPIv1_H */