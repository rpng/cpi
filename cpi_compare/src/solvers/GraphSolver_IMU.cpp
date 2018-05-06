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


#include "GraphSolver.h"


/**
 * This function will create a CPI Model 1 IMU factor.
 * This will integrate from the current state time up to the new update time
 */
ImuFactorCPIv1 GraphSolver::createimufactor_cpi_v1(double updatetime, gtsam::Values& values_initial) {

    // Get the current state and its bias estimate
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    Bias3 bg_K = stateK.bg();
    Bias3 ba_K = stateK.ba();
    JPLQuaternion q_K = stateK.q();

    // Create our preintegrator, default to analytical flags
    CpiV1 cpi(config->sigma_g,config->sigma_wg,config->sigma_a,config->sigma_wa);
    cpi.setLinearizationPoints(bg_K,ba_K,q_K,config->gravity);
    cpi.imu_avg = false;

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            cpi.feed_IMU(imu_times.at(0), imu_times.at(1), imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(1), imu_linaccs.at(1));
        }
        //cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        cpi.feed_IMU(imu_times.at(0), updatetime, imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(1), imu_linaccs.at(1));
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Debug info
    //cout << "imu dt total = " << cpi.DT << " from " << imucompound << " readings" << endl;

    // Create our factor between current and next state!
    return ImuFactorCPIv1(X(ct_state),X(ct_state+1),cpi.P_meas,cpi.DT,cpi.grav,cpi.alpha_tau,cpi.beta_tau,
                          cpi.q_k2tau,cpi.b_a_lin,cpi.b_w_lin,cpi.J_q,cpi.J_b,cpi.J_a,cpi.H_b,cpi.J_a);



}



/**
 * This function will create a CPI Model 2 IMU factor.
 * This will integrate from the current state time up to the new update time
 */
ImuFactorCPIv2 GraphSolver::createimufactor_cpi_v2(double updatetime, gtsam::Values& values_initial) {


    // Get the current state and its bias estimate
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    Bias3 bg_K = stateK.bg();
    Bias3 ba_K = stateK.ba();
    JPLQuaternion q_K = stateK.q();

    // Create our preintegrator, default to analytical flags
    CpiV2 cpi(config->sigma_g,config->sigma_wg,config->sigma_a,config->sigma_wa);
    cpi.setLinearizationPoints(bg_K,ba_K,q_K,config->gravity);
    cpi.imu_avg = false;
    cpi.state_transition_jacobians = true;

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            cpi.feed_IMU(imu_times.at(0), imu_times.at(1), imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(1), imu_linaccs.at(1));
        }
        //cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        cpi.feed_IMU(imu_times.at(0), updatetime, imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(1), imu_linaccs.at(1));
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Debug info
    //cout << "imu dt total = " << cpi.DT << " from " << imucompound << " readings" << endl;

    // Create our factor between current and next state!
    return ImuFactorCPIv2(X(ct_state),X(ct_state+1),cpi.P_meas,cpi.DT,cpi.grav,cpi.alpha_tau,cpi.beta_tau,cpi.q_k2tau,
                          cpi.q_k_lin,cpi.b_a_lin,cpi.b_w_lin,cpi.J_q,cpi.J_b,cpi.J_a,cpi.H_b,cpi.J_a,cpi.O_b,cpi.O_a);



}


/**
 * This function will create a discrete IMU factor using the GTSAM preintegrator class
 * This will integrate from the current state time up to the new update time
 */
ImuFactorCPIv1 GraphSolver::createimufactor_discrete(double updatetime, gtsam::Values& values_initial) {

    // Get the current state and its bias estimate
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    Bias3 bg_K = stateK.bg();
    Bias3 ba_K = stateK.ba();

    // Create GTSAM preintegration parameters for use with Foster's version
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    p->accelerometerCovariance = config->sigma_a_sq*Matrix33::Identity(3,3); // acc white noise in continuous
    p->gyroscopeCovariance = config->sigma_g_sq*Matrix33::Identity(3,3); // gyro white noise in continuous
    p->biasAccCovariance = config->sigma_wa_sq*Matrix33::Identity(3,3); // acc bias in continuous
    p->biasOmegaCovariance = config->sigma_wg_sq*Matrix33::Identity(3,3); // gyro bias in continuous
    p->n_gravity = -config->gravity; // gravity in navigation frame

    // Unknown noises not specified in the RSS paper
    p->integrationCovariance = 0.0*Matrix33::Identity(3,3); // error committed in integrating position from velocities
    p->biasAccOmegaInt = 0.0*Matrix66::Identity(6,6); // error in the bias used for preintegration

    // Actually create the GTSAM preintegration
    imuBias::ConstantBias prior_imu_bias(ba_K,bg_K);
    PreintegratedCombinedMeasurements preint_gtsam(p, prior_imu_bias);

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            // Our IMU measurement
            Eigen::Vector3d meas_angvel;
            Eigen::Vector3d meas_linaccs;
            meas_angvel = imu_angvel.at(0);
            meas_linaccs = imu_linaccs.at(0);
            // Preintegrate this measurement!
            preint_gtsam.integrateMeasurement(meas_linaccs, meas_angvel, dt);
        }
        //cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Our IMU measurement
        Eigen::Vector3d meas_angvel;
        Eigen::Vector3d meas_linaccs;
        meas_angvel = imu_angvel.at(0);
        meas_linaccs = imu_linaccs.at(0);
        // Preintegrate this measurement!
        preint_gtsam.integrateMeasurement(meas_linaccs, meas_angvel, dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Debug info
    //cout << "imu dt total = " << preint.Del_t << " | " << preintrk4.Del_t << " | " << preint_gtsam.deltaTij() << " from " << imucompound << " readings" << endl;

    // Get our preintegrated measurement
    // Note: We need to flip their rotation since is opposite
    Eigen::Matrix<double,3,1> alpha = preint_gtsam.deltaPij();
    Eigen::Matrix<double,3,1> beta = preint_gtsam.deltaVij();
    Eigen::Matrix<double,3,3> kplus_R_k = preint_gtsam.deltaRij().matrix().transpose();

    // Jacobians in respect to the bias
    // NOTE: Their J_q is the opposite of ours
    Eigen::Matrix<double,3,3> J_q = -preint_gtsam.delRdelBiasOmega();
    Eigen::Matrix<double,3,3> Alpha_Jacob_Accel = preint_gtsam.delPdelBiasAcc();
    Eigen::Matrix<double,3,3> Alpha_Jacob_Gyro = preint_gtsam.delPdelBiasOmega();
    Eigen::Matrix<double,3,3> Beta_Jacob_Accel = preint_gtsam.delVdelBiasAcc();
    Eigen::Matrix<double,3,3> Beta_Jacob_Gyro = preint_gtsam.delVdelBiasOmega();

    // Time interval we integrated over
    double Del_t = preint_gtsam.deltaTij();

    // Need to transform from GTSAM covariance into our state
    // CPIv1: [PreintROTATION BiasOmega PreintVELOCITY BiasAcc PreintPOSITION]
    // GTSAM: [PreintROTATION PreintPOSITION PreintVELOCITY BiasAcc BiasOmega]
    Eigen::Matrix<double,15,15> P_combined = preint_gtsam.preintMeasCov();

    //1: Swap 1 & 4 (position and bias omega)
    swapcovariance(P_combined,1,4);

    // Create our factor between current and next state!
    return ImuFactorCPIv1(X(ct_state),X(ct_state+1),P_combined,Del_t,config->gravity,
                          alpha,beta,rot_2_quat(kplus_R_k),ba_K,bg_K,J_q,
                          Beta_Jacob_Gyro,Alpha_Jacob_Gyro,Beta_Jacob_Accel,Alpha_Jacob_Accel);

}



/**
 * This will swap two entries in a covariance matrix
 * This is used to get the GTSAM covariance into our version
 */
void GraphSolver::swapcovariance(Eigen::Matrix<double,15,15>& covariance, int coli, int colj) {

    // Copy the covariance
    Eigen::Matrix<double,15,15> covariance_copy(covariance);

    // Swap two rows
    covariance_copy.block(3*coli,0,3,15) = covariance.block(3*colj,0,3,15);
    covariance_copy.block(3*colj,0,3,15) = covariance.block(3*coli,0,3,15);
    covariance = covariance_copy;

    // Swap two columns
    covariance.block(0,3*coli,15,3) = covariance_copy.block(0,3*colj,15,3);
    covariance.block(0,3*colj,15,3) = covariance_copy.block(0,3*coli,15,3);

}



/**
 * This function will get the predicted state based on the CPI measurement
 * These are based off of equation 19-23 in the tech report
 * http://udel.edu/~ghuang/papers/tr_cpi.pdf
 */
JPLNavState GraphSolver::getpredictedstate_v1(ImuFactorCPIv1& imuFactor, gtsam::Values& values_initial) {

    // Get the current state (t=k)
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    JPLQuaternion q_GtoK = stateK.q();
    Bias3 bg_K = stateK.bg();
    Velocity3 v_KinG = stateK.v();
    Bias3 ba_K = stateK.ba();
    Vector3 p_KinG = stateK.p();

    // From this we should predict where we will be at the next time (t=K+1)
    JPLQuaternion q_GtoK1 = quat_multiply(imuFactor.m_q(),q_GtoK);
    Vector3 v_K1inG = v_KinG - imuFactor.gravity()*imuFactor.dt() + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_beta();
    Vector3 p_K1inG = p_KinG + v_KinG*imuFactor.dt() - 0.5*imuFactor.gravity()*std::pow(imuFactor.dt(),2) + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_alpha();

    // Return our new state!
    return JPLNavState(q_GtoK1, bg_K, v_K1inG, ba_K, p_K1inG);

}


/**
 * This function will get the predicted state based on the CPI measurement
 * These are based off of equation 117-118 in the tech report
 * http://udel.edu/~ghuang/papers/tr_cpi.pdf
 */
JPLNavState GraphSolver::getpredictedstate_v2(ImuFactorCPIv2& imuFactor, gtsam::Values& values_initial) {

    // Get the current state (t=k)
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    JPLQuaternion q_GtoK = stateK.q();
    Bias3 bg_K = stateK.bg();
    Velocity3 v_KinG = stateK.v();
    Bias3 ba_K = stateK.ba();
    Vector3 p_KinG = stateK.p();

    // From this we should predict where we will be at the next time (t=K+1)
    JPLQuaternion q_GtoK1 = quat_multiply(imuFactor.m_q(),q_GtoK);
    Vector3 v_K1inG = v_KinG + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_beta();
    Vector3 p_K1inG = p_KinG + v_KinG*imuFactor.dt() + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_alpha();

    // Return our new state!
    return JPLNavState(q_GtoK1, bg_K, v_K1inG, ba_K, p_K1inG);

}