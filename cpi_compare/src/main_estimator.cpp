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


#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/date_time/posix_time/posix_time.hpp>


#include "solvers/GraphSolver.h"
#include "sim/SimulationLoader.h"
#include "utils/Config.h"


// Functions
void setup_config(ros::NodeHandle& nh, Config* config);
void setup_subpub(ros::NodeHandle& nh);
void handle_measurement_pose(geometry_msgs::PoseStamped::Ptr msg);
void handle_measurement_imu(sensor_msgs::Imu::Ptr msg);
void handle_measurement_uv(cpi_comm::CameraMeasurement::Ptr msg);
void publish_JPLstate(double timestamp, JPLNavState& state, Eigen::Matrix<double, 6, 6>& covariance,
                      ros::Publisher& pubPath, ros::Publisher& pubPoseIMU, vector<geometry_msgs::PoseStamped>& poses_est);
void publish_FeatureCloud(double timestamp);


// Subscribers and publishers
ros::Publisher pubFeatureCloudsv1;
ros::Publisher pubFeatureCloudsv2;
ros::Publisher pubFeatureCloudsFORSTER;
ros::Publisher pubPathMODEL1;
ros::Publisher pubPathMODEL2;
ros::Publisher pubPathFORSTER;
ros::Publisher pubPoseIMUMODEL1;
ros::Publisher pubPoseIMUMODEL2;
ros::Publisher pubPoseIMUFORSTER;
ros::Subscriber subUVMeas;
ros::Subscriber subIMUMeas;
ros::Subscriber subPOSETrue;

// Master config and graph object
Config* config;
GraphSolver* graphsolver;

// Variables needed for visualization
boost::thread* t_viz;
unsigned int poses_seq = 0;
vector<geometry_msgs::PoseStamped> poses_estMODEL1;
vector<geometry_msgs::PoseStamped> poses_estMODEL2;
vector<geometry_msgs::PoseStamped> poses_estFORSTER;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Setup our config object
    config = new Config();
    setup_config(nhPrivate, config);

    // Sleep a bit to let publishers setup
    sleep(2);

    // Next up, setup our subscribers and publishers
    setup_subpub(nh);

    // Create our gtsam estimation class
    graphsolver = new GraphSolver(config);

    // Create our visualization thread
    //t_viz = new boost::thread(boost::bind(&thread_visualization));

    // Let ROS handle callbacks if not doing serial
    if(!config->useSerialSIM) {
        ros::spin();
    } else {
        // Create simulator loader
        SimulationLoader sim(config->simPath,nh,nhPrivate);
        // Lets do the simulation!
        sim.execute_publishing(&handle_measurement_pose,&handle_measurement_imu,&handle_measurement_uv);
    }

    // Shutdown subscribers
    pubFeatureCloudsv1.shutdown();
    pubFeatureCloudsv2.shutdown();
    pubFeatureCloudsFORSTER.shutdown();
    pubPathMODEL1.shutdown();
    pubPathMODEL2.shutdown();
    pubPathFORSTER.shutdown();
    pubPoseIMUMODEL1.shutdown();
    pubPoseIMUMODEL2.shutdown();
    pubPoseIMUFORSTER.shutdown();
    subPOSETrue.shutdown();
    subIMUMeas.shutdown();
    subUVMeas.shutdown();

    // Done!
    return EXIT_SUCCESS;
}




/**
 * \brief This function will read in config values from the node handler
 * If the values are not found, then use a default for each one.
 */
void setup_config(ros::NodeHandle& nh, Config* config) {

    // Read in our CAMERA noise values
    nh.param<double>("sigma_camera", config->sigma_camera, 1.0/484.1316);
    config->sigma_camera_sq = std::pow(config->sigma_camera,2);

    // Read in our IMU noise values
    nh.param<double>("gyroscope_noise_density", config->sigma_g, 0.005);
    config->sigma_g_sq = std::pow(config->sigma_g,2);
    nh.param<double>("accelerometer_noise_density", config->sigma_a, 0.01);
    config->sigma_a_sq = std::pow(config->sigma_a,2);
    nh.param<double>("gyroscope_random_walk", config->sigma_wg, 4.0e-06);
    config->sigma_wg_sq = std::pow(config->sigma_wg,2);
    nh.param<double>("accelerometer_random_walk", config->sigma_wa, 0.002);
    config->sigma_wa_sq = std::pow(config->sigma_wa,2);

    // Load global gravity
    std::vector<double> gravity = {0,0,9.8};
    nh.param<std::vector<double>>("gravity", gravity, gravity);
    for(size_t i=0;i<3;i++) config->gravity(i,0) = gravity.at(i);

    // SIMULATION: Simulation config for the transform
    // TODO: read from launch file!!!
    config->R_C0toI << 1,0,0,0,1,0,0,0,1;
    //config->R_C0toI << 0,0,1,0,1,0,-1,0,0; //-90 about the y-axis
    config->p_IinC0 << 0,0,0;
    config->R_C1toI << 1,0,0,0,1,0,0,0,1;
    //config->R_C1toI << 0,0,1,0,1,0,-1,0,0; //-90 about the y-axis
    config->p_IinC1 << 0.1,0,0;
    config->p_IinC1 = -config->R_C1toI.transpose()*config->p_IinC1;

    // Load if we should do a serial simulation
    nh.param<bool>("useSerialSIM", config->useSerialSIM, true);

    // Load simulation path
    config->simPath = "";
    nh.param<std::string>("simPath", config->simPath, config->simPath);

    // Load if we should inialize using the groundtruth pose
    nh.param<bool>("useGroundTruthInit", config->useGroundTruthInit, true);

    // Max window we should update uv coordinates by
    nh.param<int>("uvWindowSize", config->uvWindowSize, 25);

    // Load if we should use inverse depth for feature measurements
    nh.param<bool>("useInverseDepth", config->useInverseDepth, true);

    // Number of poses we should see a feature from before we initialize it
    nh.param<int>("minPoseFeatureInit", config->minPoseFeatureInit, 5);

    // Amount of seconds we should smooth by
    nh.param<int>("lagSmootherAmount", config->lagSmootherAmount, 3);

    // Read in IMU rates (hertz) and number of IMU message we should wait to initialize from
    nh.param<int>("imurate", config->imuRate, 100);
    nh.param<int>("camrate", config->camRate, 10);
    nh.param<int>("imuwait", config->imuWait, 300);

    // Debug print to screen for the user
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    cout << "Noise Parameters:" << endl;
    cout << "\t- sigma_p:  " << config->sigma_camera << endl;
    cout << "\t- sigma_g:  " << config->sigma_g << endl;
    cout << "\t- sigma_wg: " << config->sigma_wg << endl;
    cout << "\t- sigma_a:  " << config->sigma_a << endl;
    cout << "\t- sigma_wa: " << config->sigma_wa << endl;
    cout << "World Parameters:" << endl;
    cout << "\t- imuRate: " << config->imuRate << endl;
    cout << "\t- camRate: " << config->camRate << endl;
    cout << "\t- imuWait: " << config->imuWait << endl;
    cout << "\t- gravity: " << config->gravity.format(CommaInitFmt) << endl;
    cout << "Transform Parameters:" << endl;
    cout << "\t- R_C0toI: " << endl << config->R_C0toI << endl;
    cout << "\t- p_IinC0: " << endl << config->p_IinC0.transpose() << endl;
    cout << "\t- R_C1toI: " << endl << config->R_C1toI << endl;
    cout << "\t- p_IinC1: " << endl << config->p_IinC1.transpose() << endl;
    cout << "General Parameters:" << endl;
    cout << "\t- lag amount (s): " << config->lagSmootherAmount << endl;
    cout << "\t- inverse depth: " << std::boolalpha << config->useInverseDepth << endl;
    cout << "\t- groundtruth init: " << std::boolalpha << config->useGroundTruthInit << endl;

}




/**
 * \brief This performs the setup for all the ROS subscriber and publishers needed
 */
void setup_subpub(ros::NodeHandle& nh) {

    // Point cloud visualization
    pubFeatureCloudsv1 = nh.advertise<sensor_msgs::PointCloud2>("cpi_compare/feature_cloudv1", 2);
    ROS_INFO("Publishing: %s", pubFeatureCloudsv1.getTopic().c_str());
    pubFeatureCloudsv2 = nh.advertise<sensor_msgs::PointCloud2>("cpi_compare/feature_cloudv2", 2);
    ROS_INFO("Publishing: %s", pubFeatureCloudsv2.getTopic().c_str());
    pubFeatureCloudsFORSTER = nh.advertise<sensor_msgs::PointCloud2>("cpi_compare/feature_cloud_forster", 2);
    ROS_INFO("Publishing: %s", pubFeatureCloudsFORSTER.getTopic().c_str());

    // Path visualization
    pubPathMODEL1 = nh.advertise<nav_msgs::Path>("cpi_compare/path_imu_cpi1", 2);
    ROS_INFO("Publishing: %s", pubPathMODEL1.getTopic().c_str());
    pubPathMODEL2 = nh.advertise<nav_msgs::Path>("cpi_compare/path_imu_cpi2", 2);
    ROS_INFO("Publishing: %s", pubPathMODEL2.getTopic().c_str());
    pubPathFORSTER = nh.advertise<nav_msgs::Path>("cpi_compare/path_imu_forster", 2);
    ROS_INFO("Publishing: %s", pubPathFORSTER.getTopic().c_str());

    // IMU pose visualization
    pubPoseIMUMODEL1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("cpi_compare/pose_imu_cpi1", 2);
    ROS_INFO("Publishing: %s", pubPoseIMUMODEL1.getTopic().c_str());
    pubPoseIMUMODEL2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("cpi_compare/pose_imu_cpi2", 2);
    ROS_INFO("Publishing: %s", pubPoseIMUMODEL2.getTopic().c_str());
    pubPoseIMUFORSTER = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("cpi_compare/pose_imu_forster", 2);
    ROS_INFO("Publishing: %s", pubPoseIMUFORSTER.getTopic().c_str());

    // Subscribe to our true pose measurements
    //subPOSETrue = nh.subscribe("cpi_compare/truepose_imu", 2000, handle_measurement_pose);
    //ROS_INFO("Subscribing: %s", subPOSETrue.getTopic().c_str());

    // Subscribe to our IMU measurements
    //subIMUMeas = nh.subscribe("cpi_compare/data_imu", 2000, handle_measurement_imu);
    //ROS_INFO("Subscribing: %s", subIMUMeas.getTopic().c_str());

    // Subscribe to the uv measurements
    //subUVMeas = nh.subscribe("cpi_compare/data_uv", 500, handle_measurement_uv);
    //ROS_INFO("Subscribing: %s", subUVMeas.getTopic().c_str());

}



/**
 * \brief Subscription callback for TRUE IMU POSE messages from the simulator
 * @param msg TRUE POSE message
 */
void handle_measurement_pose(geometry_msgs::PoseStamped::Ptr msg) {

    // Convert to eigen format
    Eigen::Vector4d q_GtoI;
    q_GtoI << msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w;
    Eigen::Vector3d p_IinG;
    p_IinG << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;

    // Send to our graph solver
    graphsolver->addtrue_pose(msg->header.stamp.toSec(),q_GtoI,p_IinG);

}


/**
 * \brief Subscription callback for IMU messages from the sensor
 * @param msg IMU message
 */
void handle_measurement_imu(sensor_msgs::Imu::Ptr msg) {

    // Convert to eigen format
    Eigen::Vector3d linearacceleration;
    linearacceleration << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    Eigen::Vector3d angularvelocity;
    angularvelocity << msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;

    // Send to our graph solver
    graphsolver->addmeasurement_imu(msg->header.stamp.toSec(),linearacceleration,angularvelocity);

}



/**
 * \brief Subscription callback for UV coordinate measurements
 * @param msg CameraMeasurement message
 */
void handle_measurement_uv(cpi_comm::CameraMeasurement::Ptr msg) {

    //==========================================================================
    // PROCESS NEW MEASUREMENTS
    //==========================================================================

    // Our feature measurements
    std::vector<uint> leftids;
    std::vector<Eigen::Vector2d> leftuv;
    std::vector<uint> rightids;
    std::vector<Eigen::Vector2d> rightuv;

    // Loop through LEFT features and append
    for(size_t i=0; i<msg->features_left.size(); i++) {
        leftids.push_back((uint)msg->features_left.at(i).id);
        Eigen::Vector2d uv;
        uv << msg->features_left.at(i).u, msg->features_left.at(i).v;
        leftuv.push_back(uv);
    }

    // Loop through RIGHT features and append
    for(size_t i=0; i<msg->features_right.size(); i++) {
        rightids.push_back((uint)msg->features_right.at(i).id);
        Eigen::Vector2d uv;
        uv << msg->features_right.at(i).u, msg->features_right.at(i).v;
        rightuv.push_back(uv);
    }

    // We have successfully handled all features, lets send them to the optimizer
    graphsolver->addmeasurement_uv(msg->header.stamp.toSec(),leftids,leftuv,rightids,rightuv);

    //==========================================================================
    // OPTIMIZE GRAPH
    //==========================================================================

    // Optimize the graph! GO GO GO!!!
    graphsolver->optimize();


    //==========================================================================
    // ROS VISUALIZATION
    //==========================================================================

    // Temp empty covariance, since this takes a while to compute
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double,6,6>::Zero();

    // CPI MODEL 1
    JPLNavState state = graphsolver->getcurrentstateMODEL1();
    publish_JPLstate(msg->header.stamp.toSec(),state,covariance,pubPathMODEL1,pubPoseIMUMODEL1,poses_estMODEL1);

    // CPI MODEL 2
    state = graphsolver->getcurrentstateMODEL2();
    publish_JPLstate(msg->header.stamp.toSec(),state,covariance,pubPathMODEL2,pubPoseIMUMODEL2,poses_estMODEL2);

    // FORSTER DISCRETE
    state = graphsolver->getcurrentstateFORSTER();
    publish_JPLstate(msg->header.stamp.toSec(),state,covariance,pubPathFORSTER,pubPoseIMUFORSTER,poses_estFORSTER);

    // Publish our feature cloud
    publish_FeatureCloud(msg->header.stamp.toSec());

}




/**
 * Given a JPL Navigation State, this should publish it onto ROS for visualization
 */
void publish_JPLstate(double timestamp, JPLNavState& state, Eigen::Matrix<double, 6, 6>& covariance,
                      ros::Publisher& pubPath, ros::Publisher& pubPoseIMU, vector<geometry_msgs::PoseStamped>& poses_est) {

    // Return if we have not initialized yet
    if(!graphsolver->is_initialized())
        return;

    // Create our stamped pose with covariance
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ros::Time(timestamp);
    pose.header.frame_id = "global";
    pose.pose.pose.orientation.x = state.q()(0);
    pose.pose.pose.orientation.y = state.q()(1);
    pose.pose.pose.orientation.z = state.q()(2);
    pose.pose.pose.orientation.w = state.q()(3);
    pose.pose.pose.position.x = state.p()(0);
    pose.pose.pose.position.y = state.p()(1);
    pose.pose.pose.position.z = state.p()(2);

    // Finally set the covariance in the message
    pose.pose.covariance[0] = covariance(0,0); // 0
    pose.pose.covariance[1] = covariance(0,1);
    pose.pose.covariance[2] = covariance(0,2);
    pose.pose.covariance[3] = covariance(0,3);
    pose.pose.covariance[4] = covariance(0,4);
    pose.pose.covariance[5] = covariance(0,5);
    pose.pose.covariance[6] = covariance(1,0); // 1
    pose.pose.covariance[7] = covariance(1,1);
    pose.pose.covariance[8] = covariance(1,2);
    pose.pose.covariance[9] = covariance(1,3);
    pose.pose.covariance[10] = covariance(1,4);
    pose.pose.covariance[11] = covariance(1,5);
    pose.pose.covariance[12] = covariance(2,0); // 2
    pose.pose.covariance[13] = covariance(2,1);
    pose.pose.covariance[14] = covariance(2,2);
    pose.pose.covariance[15] = covariance(2,3);
    pose.pose.covariance[16] = covariance(2,4);
    pose.pose.covariance[17] = covariance(2,5);
    pose.pose.covariance[18] = covariance(3,0); // 3
    pose.pose.covariance[19] = covariance(3,1);
    pose.pose.covariance[20] = covariance(3,2);
    pose.pose.covariance[21] = covariance(3,3);
    pose.pose.covariance[22] = covariance(3,4);
    pose.pose.covariance[23] = covariance(3,5);
    pose.pose.covariance[24] = covariance(4,0); // 4
    pose.pose.covariance[25] = covariance(4,1);
    pose.pose.covariance[26] = covariance(4,2);
    pose.pose.covariance[27] = covariance(4,3);
    pose.pose.covariance[28] = covariance(4,4);
    pose.pose.covariance[29] = covariance(4,5);
    pose.pose.covariance[30] = covariance(5,0); // 5
    pose.pose.covariance[31] = covariance(5,1);
    pose.pose.covariance[32] = covariance(5,2);
    pose.pose.covariance[33] = covariance(5,3);
    pose.pose.covariance[34] = covariance(5,4);
    pose.pose.covariance[35] = covariance(5,5);

    // Publish this pose
    pubPoseIMU.publish(pose);

    // Create our stamped pose for path publishing
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(timestamp);
    poseStamped.header.frame_id = "global";
    poseStamped.pose = pose.pose.pose;
    poses_est.push_back(poseStamped);

    // Create pose arrays and publish
    nav_msgs::Path patharr;
    patharr.header.stamp = ros::Time(timestamp);
    patharr.header.seq = poses_seq++;
    patharr.header.frame_id = "global";
    patharr.poses = poses_est;
    pubPath.publish(patharr);


    // Debug print current position
    ROS_INFO("[STATE]: q = %.4f, %.4f, %.4f, %.4f | v = %.2f, %.2f, %.2f | p = %.2f, %.2f, %.2f",
             state.q()(0), state.q()(1), state.q()(2), state.q()(3), state.v()(0), state.v()(1),
             state.v()(2), state.p()(0), state.p()(1), state.p()(2));

}




/**
 * Given a set of point clouds, this should publish them onto ROS
 * NOTE: POINTS SHOULD ALREADY BE IN THE GLOBAL FRAME OF REFERENCE!!!!@@!#
 */
void publish_FeatureCloud(double timestamp) {

    // Return if we have not initialized yet
    if(!graphsolver->is_initialized())
        return;

    // Loop through and create a cloud
    std::vector<Eigen::Vector3d> points = graphsolver->getcurrentfeaturesMODEL1();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    for(size_t i=0; i<points.size(); i++) {
        pcl::PointXYZ pt;
        pt.x = points.at(i)(0);
        pt.y = points.at(i)(1);
        pt.z = points.at(i)(2);
        cloud->push_back(pt);
    }
    std::vector<Eigen::Vector3d> points2 = graphsolver->getcurrentfeaturesMODEL2();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud <pcl::PointXYZ>);
    for(size_t i=0; i<points2.size(); i++) {
        pcl::PointXYZ pt;
        pt.x = points2.at(i)(0);
        pt.y = points2.at(i)(1);
        pt.z = points2.at(i)(2);
        cloud2->push_back(pt);
    }

    std::vector<Eigen::Vector3d> points3 = graphsolver->getcurrentfeaturesFORSTER();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud <pcl::PointXYZ>);
    for(size_t i=0; i<points2.size(); i++) {
        pcl::PointXYZ pt;
        pt.x = points3.at(i)(0);
        pt.y = points3.at(i)(1);
        pt.z = points3.at(i)(2);
        cloud3->push_back(pt);
    }

    // Publish the downstampled cloud
    sensor_msgs::PointCloud2 msgOut;
    pcl::toROSMsg(*cloud, msgOut);
    msgOut.header.frame_id = "global";
    msgOut.header.stamp = ros::Time(timestamp);
    pubFeatureCloudsv1.publish(msgOut);

    // Publish the downstampled cloud
    sensor_msgs::PointCloud2 msgOut2;
    pcl::toROSMsg(*cloud2, msgOut2);
    msgOut2.header.frame_id = "global";
    msgOut2.header.stamp = ros::Time(timestamp);
    pubFeatureCloudsv2.publish(msgOut2);

    // Publish the downstampled cloud
    sensor_msgs::PointCloud2 msgOut3;
    pcl::toROSMsg(*cloud3, msgOut3);
    msgOut3.header.frame_id = "global";
    msgOut3.header.stamp = ros::Time(timestamp);
    pubFeatureCloudsFORSTER.publish(msgOut3);

}
