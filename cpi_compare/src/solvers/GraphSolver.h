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


#ifndef GRAPHSOLVER_H
#define GRAPHSOLVER_H

#include <mutex>
#include <thread>
#include <deque>
#include <fstream>
#include <unordered_map>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "gtsam/ImuFactorCPIv1.h"
#include "gtsam/ImuFactorCPIv2.h"
#include "gtsam/JPLNavState.h"
#include "gtsam/JPLNavStatePrior.h"
#include "gtsam/JPLImageUVFactor.h"
#include "gtsam/InvAnchorFactor.h"
#include "gtsam/InvUVFactor.h"
#include "solvers/FeatureInitializer.h"
#include "utils/quat_ops.h"
#include "utils/Config.h"
#include "utils/feature.h"

#include "cpi/CpiV1.h"
#include "cpi/CpiV2.h"

using namespace std;
using namespace gtsam;


using gtsam::symbol_shorthand::X; // X: our JPL states
using gtsam::symbol_shorthand::F; // F: our feature node


class GraphSolver {
public:


    /**
     * Default constructor
     */
    GraphSolver(Config* config) {
        // Store our config object
        this->config = config;
        // Initalize our graphs
        this->graphMODEL1 = new gtsam::NonlinearFactorGraph();
        this->graphMODEL2 = new gtsam::NonlinearFactorGraph();
        this->graphFORSTER = new gtsam::NonlinearFactorGraph();
        this->graph_newMODEL1 = new gtsam::NonlinearFactorGraph();
        this->graph_newMODEL2 = new gtsam::NonlinearFactorGraph();
        this->graph_newFORSTER = new gtsam::NonlinearFactorGraph();
        // Fixed lag smoothers BATCH
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
        this->smootherBatchMODEL1 = new BatchFixedLagSmoother(config->lagSmootherAmount,params,true);
        this->smootherBatchMODEL2 = new BatchFixedLagSmoother(config->lagSmootherAmount,params,true);
        this->smootherBatchFORSTER = new BatchFixedLagSmoother(config->lagSmootherAmount,params,true);

    }

    /// Function that takes in IMU measurements for use in preintegration measurements
    void addtrue_pose(double timestamp, Eigen::Vector4d q_GtoI, Eigen::Vector3d p_IinG);

    /// Function that takes in IMU measurements for use in preintegration measurements
    void addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel);

    /// Function that takes in UV measurements that will be used as "features" in our graph
    void addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                           std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);

    /// This function will optimize the graph
    void optimize();

    /// Will return true if the system is initialized
    bool is_initialized() {
        return systeminitalized;
    }

    /// This function returns the current nav state, return origin if we have not initialized yet
    JPLNavState getcurrentstateMODEL1() {
        if(values_initialMODEL1.empty())
            return JPLNavState();
        return values_initialMODEL1.at<JPLNavState>(X(ct_state));
    }

    /// This function returns the current nav state, return origin if we have not initialized yet
    JPLNavState getcurrentstateMODEL2() {
        if(values_initialMODEL2.empty())
            return JPLNavState();
        return values_initialMODEL2.at<JPLNavState>(X(ct_state));
    }

    /// This function returns the current nav state, return origin if we have not initialized yet
    JPLNavState getcurrentstateFORSTER() {
        if(values_initialFORSTER.empty())
            return JPLNavState();
        return values_initialFORSTER.at<JPLNavState>(X(ct_state));
    }

    /// Returns the currently tracked features
    std::vector<Eigen::Vector3d> getcurrentfeaturesMODEL1() {
        // Return if we do not have any nodes yet
        if(values_initialMODEL1.empty()) {
            return std::vector<Eigen::Vector3d>();
        }
        // Our vector of points in the global
        std::vector<Eigen::Vector3d> features;
        // Else loop through the features and return them
        for(size_t i=1; i<= ct_features; i++) {
            // Ensure valid feature
            if(!values_initialMODEL1.exists(F(i)))
                continue;
            // If not doing inverse depth, just directly add the feature
            if(!config->useInverseDepth) {
                features.push_back(values_initialMODEL1.at<Point3>(F(i)));
                continue;
            }
            // If inverse depth, need to transform into global
            if(!values_initialMODEL1.exists(X(measurement_anchor_lookup[i])))
                continue;
            // Transform back into the global frame!
            JPLNavState state = values_initialMODEL1.at<JPLNavState>(X(measurement_anchor_lookup[i]));
            Eigen::Vector3d abr = values_initialMODEL1.at<Point3>(F(i));
            Eigen::Vector3d p_FinA;
            p_FinA << abr(0)/abr(2), abr(1)/abr(2), 1/abr(2);
            Eigen::Vector3d p_FinG = quat_2_Rot(state.q()).transpose()*p_FinA+state.p();
            features.push_back(p_FinG);
        }
        return features;
    }

    /// Returns the currently tracked features
    std::vector<Eigen::Vector3d> getcurrentfeaturesMODEL2() {
        // Return if we do not have any nodes yet
        if(values_initialMODEL2.empty()) {
            return std::vector<Eigen::Vector3d>();
        }
        // Our vector of points in the global
        std::vector<Eigen::Vector3d> features;
        // Else loop through the features and return them
        for(size_t i=1; i<= ct_features; i++) {
            // Ensure valid feature
            if(!values_initialMODEL2.exists(F(i)))
                continue;
            // If not doing inverse depth, just directly add the feature
            if(!config->useInverseDepth) {
                features.push_back(values_initialMODEL2.at<Point3>(F(i)));
                continue;
            }
            // If inverse depth, need to transform into global
            if(!values_initialMODEL2.exists(X(measurement_anchor_lookup[i])))
                continue;
            // Transform back into the global frame!
            JPLNavState state = values_initialMODEL2.at<JPLNavState>(X(measurement_anchor_lookup[i]));
            Eigen::Vector3d abr = values_initialMODEL2.at<Point3>(F(i));
            Eigen::Vector3d p_FinA;
            p_FinA << abr(0)/abr(2), abr(1)/abr(2), 1/abr(2);
            Eigen::Vector3d p_FinG = quat_2_Rot(state.q()).transpose()*p_FinA+state.p();
            features.push_back(p_FinG);
        }
        return features;
    }

    /// Returns the currently tracked features
    std::vector<Eigen::Vector3d> getcurrentfeaturesFORSTER() {
        // Return if we do not have any nodes yet
        if(values_initialFORSTER.empty()) {
            return std::vector<Eigen::Vector3d>();
        }
        // Our vector of points in the global
        std::vector<Eigen::Vector3d> features;
        // Else loop through the features and return them
        for(size_t i=1; i<= ct_features; i++) {
            // Ensure valid feature
            if(!values_initialFORSTER.exists(F(i)))
                continue;
            // If not doing inverse depth, just directly add the feature
            if(!config->useInverseDepth) {
                features.push_back(values_initialFORSTER.at<Point3>(F(i)));
                continue;
            }
            // If inverse depth, need to transform into global
            if(!values_initialFORSTER.exists(X(measurement_anchor_lookup[i])))
                continue;
            // Transform back into the global frame!
            JPLNavState state = values_initialFORSTER.at<JPLNavState>(X(measurement_anchor_lookup[i]));
            Eigen::Vector3d abr = values_initialFORSTER.at<Point3>(F(i));
            Eigen::Vector3d p_FinA;
            p_FinA << abr(0)/abr(2), abr(1)/abr(2), 1/abr(2);
            Eigen::Vector3d p_FinG = quat_2_Rot(state.q()).transpose()*p_FinA+state.p();
            features.push_back(p_FinG);
        }
        return features;
    }



private:

    /// Function which will try to initalize our graph using the current IMU measurements
    void trytoinitalize(double timestamp);

    /// Function that will compound our IMU measurements up to the given timestep
    ImuFactorCPIv1 createimufactor_cpi_v1(double updatetime, gtsam::Values& values_initial);
    ImuFactorCPIv2 createimufactor_cpi_v2(double updatetime, gtsam::Values& values_initial);

    /// Function that will compound the GTSAM preintegrator to get discrete preintegration measurement
    ImuFactorCPIv1 createimufactor_discrete(double updatetime, gtsam::Values& values_initial);

    /// Function will get the predicted JPL Navigation State based on this generated measurement
    JPLNavState getpredictedstate_v1(ImuFactorCPIv1& imuFactor, gtsam::Values& values_initial);
    JPLNavState getpredictedstate_v2(ImuFactorCPIv2& imuFactor, gtsam::Values& values_initial);

    /// Column swap
    void swapcovariance(Eigen::Matrix<double,15,15>& covariance, int coli, int colj);

    /// Normal feature measurements
    void process_feat_normal(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                             std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);

    /// Inverse feature measurements
    void process_feat_inverse(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                              std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);



    //==========================================================================
    // OPTIMIZATION OBJECTS
    //==========================================================================

    // Master non-linear GTSAM graph, all created factors
    gtsam::NonlinearFactorGraph* graphMODEL1;
    gtsam::NonlinearFactorGraph* graphMODEL2;
    gtsam::NonlinearFactorGraph* graphFORSTER;

    // New factors that have not been optimized yet
    gtsam::NonlinearFactorGraph* graph_newMODEL1;
    gtsam::NonlinearFactorGraph* graph_newMODEL2;
    gtsam::NonlinearFactorGraph* graph_newFORSTER;

    // Fixed lag smothers objects
    BatchFixedLagSmoother* smootherBatchMODEL1;
    BatchFixedLagSmoother* smootherBatchMODEL2;
    BatchFixedLagSmoother* smootherBatchFORSTER;

    // Timestamps of the current nodes in our graph
    FixedLagSmoother::KeyTimestampMap newTimestampsMODEL1;
    FixedLagSmoother::KeyTimestampMap newTimestampsMODEL2;
    FixedLagSmoother::KeyTimestampMap newTimestampsFORSTER;

    // Current ID of state and features
    size_t ct_state = 0;
    size_t ct_features = 0;

    // All created nodes
    gtsam::Values values_initialMODEL1;
    gtsam::Values values_initialMODEL2;
    gtsam::Values values_initialFORSTER;

    // New nodes that have not been optimized
    gtsam::Values values_newMODEL1;
    gtsam::Values values_newMODEL2;
    gtsam::Values values_newFORSTER;

    //==========================================================================
    // SYSTEM / HOUSEKEEPING VARIABLES
    //==========================================================================

    /// Our config object (has all sensor noise values)
    Config* config;

    /// Boolean that tracks if we have initialized
    bool systeminitalized = false;

    // Our true POSE data
    std::mutex truth_mutex;
    std::deque<double> true_times;
    std::deque<Eigen::Vector4d> true_qGtoI;
    std::deque<Eigen::Vector3d> true_pIinG;

    // Our IMU data from the sensor
    std::mutex imu_mutex;
    std::deque<double> imu_times;
    std::deque<Eigen::Vector3d> imu_linaccs;
    std::deque<Eigen::Vector3d> imu_angvel;

    /// Lookup tables for features and incoming measurements
    std::mutex features_mutex;
    std::unordered_map<int, size_t> measurement_lookup; //< node ID of feature if added into graph
    std::unordered_map<int, size_t> measurement_state_lookup; //< state ID of feature if added into graph
    std::unordered_map<size_t, size_t> measurement_anchor_lookup; //< state ID of anchor pose based on feature ID
    std::unordered_map<int, feature> measurement_queue; //< queue of features that have not been added


};



#endif /* GRAPHSOLVER_H */
