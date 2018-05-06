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


#ifndef GTSAM_INVANCHORFACTOR_H
#define GTSAM_INVANCHORFACTOR_H

#include <gtsam/base/debug.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "JPLNavState.h"
#include "utils/quat_ops.h"

using namespace gtsam;

namespace gtsam {


    /**
     * \brief Inverse Image UV Factor
     * Links a JPl NavState and a 3D feature in the environment
     * Uses the inverse depth representation, this is for the anchoring measurement
     * NOTE: anchor only needs to link to the feature node
     */
    class InvAnchorFactor : public NoiseModelFactor1<Point3> {
    private:

        Eigen::Vector2d uv; ///< normalized uv coordinates

        Eigen::Matrix3d R_CtoI; ///< static rotation from the IMU to this camera frame
        Eigen::Vector3d p_IinC; ///< static transform from the IMU to this camera frame

        Eigen::Matrix3d R_CAtoI; ///< static rotation from the IMU to the ANCHOR camera frame
        Eigen::Vector3d p_IinCA; ///< static transform from the IMU to the ANCHOR camera frame

    public:

        /// Construct from the a JPLNavState and feature Point3, preingration measurement, and its covariance
        InvAnchorFactor(Key state_i, Eigen::Matrix<double,2,2> covariance, Eigen::Vector2d& uvi, Eigen::Matrix3d& R_CtoIi, Eigen::Vector3d& p_IinCi, Eigen::Matrix3d& R_CAtoIi, Eigen::Vector3d& p_IinCAi) :
                NoiseModelFactor1<Point3>(noiseModel::Gaussian::Covariance(covariance), state_i),
                uv(uvi), R_CtoI(R_CtoIi), p_IinC(p_IinCi), R_CAtoI(R_CAtoIi), p_IinCA(p_IinCAi) { }


        /// Return uv coordinates
        Vector2 uvcoords() const {
            return uv;
        }


        /// Error function. Given the current states, calculate the measurement error/residual
        gtsam::Vector evaluateError(const Point3& state_i, boost::optional<Matrix&> H1 = boost::none) const;


        /// How this factor gets printed in the ostream
        GTSAM_EXPORT
        friend std::ostream &operator<<(std::ostream &os, const InvAnchorFactor& factor) {
            os << "uv:[" << factor.uvcoords()(0) << ", " << factor.uvcoords()(1) << "]'" << endl;
            return os;
        }

        /// Print function for this factor
        void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "InvAnchorFactor(" << keyFormatter(this->key()) << ")" << std::endl;
            std::cout << "  measured: " << std::endl << *this << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /// Define how two factors can be equal to each other
        bool equals(const NonlinearFactor &expected, double tol = 1e-9) const {
            // Cast the object
            const InvAnchorFactor *e =  dynamic_cast<const InvAnchorFactor*>(&expected);
            if(e == NULL) return false;
            // Success, compare base noise values and the measurement values
            return NoiseModelFactor1<Point3>::equals(*e, tol)
                   && gtsam::equal(uv, e->uv, tol)
                   && gtsam::equal(R_CtoI, e->R_CtoI, tol)
                   && gtsam::equal(p_IinC, e->p_IinC, tol);
        }

    };


} // namespace gtsam


#endif /* GTSAM_INVANCHORFACTOR_H */