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


#include "SimulationLoader.h"


/**
 * \brief This will try to load all the key files in a given simulation folder
 * \brief It will error if it is missing a file in the given path
 *
 * @param path Path to the folder that has the simulation txt files
 */
void SimulationLoader::load_files(std::string path) {

    // Base ROS time we will add all the simulated times too
    // This is fixed since we want each simulation to at the same time with different noise
    ros::Time basetime = ros::Time(1519060444);
    //ros::Time basetime = ros::Time::now();

    // Master file stream object
    std::ifstream file01;
    std::ifstream file02;
    std::ifstream file03;
    std::ifstream file04;
    std::string line01;
    std::string line02;
    std::string line03;
    std::string line04;


    //======================================================================
    // READ THE TRUE IMU POSE FILE
    file01.open(path+"pose_true.dat");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"pose_true.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        // Get the pose for this line
        geometry_msgs::PoseStamped pose;
        pose = SimParser::parsePoseLine(basetime, "global", line01);
        // Append it to our vector of true poses
        poses_imu.push_back(pose);
    }
    file01.close();

    //======================================================================
    // READ THE TRUE IMU MEASUREMENTS FILE
    file01.open(path+"imu_data_meas.dat");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"imu_data_meas.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        // Get the measurement for this line
        sensor_msgs::Imu meas_noisy;
        meas_noisy = SimParser::parseImuMeasurementLine(basetime, "imu", line01);
        // Append it to our vector of measurements
        measurements_imu.push_back(meas_noisy);
    }
    file01.close();


    //======================================================================
    // READ THE TRUE IMU MEASUREMENTS FILE
    file01.open(path+"camera_data_meas.dat");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"camera_data_meas.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    file02.open(path+"camera_data_id.dat");
    if (!file02) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"camera_data_id.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    file03.open(path+"camera2_data_meas.dat");
    if (!file03) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"camera2_data_meas.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    file04.open(path+"camera2_data_id.dat");
    if (!file04) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"camera2_data_id.dat").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && std::getline(file02, line02) && std::getline(file03, line03) && std::getline(file04, line04) && ros::ok()) {
        // Get the measurement for this line
        cpi_comm::CameraMeasurement meas_noisy;
        meas_noisy = SimParser::parseCameraMeasurementLine(basetime, "cam", line01, line02, line03, line04);
        // Append it to our vector of measurements
        measurements_camera.push_back(meas_noisy);
    }
    file01.close();
    file02.close();
    file03.close();
    file04.close();

    // Remove last measurement since they are all zeros
    poses_imu.pop_back();
    measurements_imu.pop_back();
    measurements_camera.pop_back();


    // Debug information
    ROS_INFO("LOAD: %d true IMU poses",(int)poses_imu.size());
    ROS_INFO("LOAD: %d meas IMU measurements",(int)measurements_imu.size());
    ROS_INFO("LOAD: %d true CAM poses",(int)poses_cam.size());
    ROS_INFO("LOAD: %d meas CAM measurements",(int)measurements_camera.size());


    // Check that we at least have one measurement of each
    if(poses_imu.empty() || measurements_imu.empty() || measurements_camera.empty()) {
        ROS_ERROR("ERROR: Need at least one of each type of measurement...cannot continue.");
        std::exit(EXIT_FAILURE);
    }


}



/**
 * \brief This performs the setup for all the ROS publishers needed
 */
void SimulationLoader::setup_publishers(ros::NodeHandle nh) {

    // ROS TF
    mTfBr = new tf::TransformBroadcaster();

    // True pose information
    pubPoseIMU = nh.advertise<geometry_msgs::PoseStamped>("cpi_compare/truepose_imu", 999);
    ROS_INFO("Publishing: %s", pubPoseIMU.getTopic().c_str());
    pubPoseCAM0 = nh.advertise<geometry_msgs::PoseStamped>("cpi_compare/truepose_cam0", 999);
    ROS_INFO("Publishing: %s", pubPoseCAM0.getTopic().c_str());

    // True pose path
    pubPathIMU = nh.advertise<nav_msgs::Path>("cpi_compare/truepath_imu", 999);
    ROS_INFO("Publishing: %s", pubPathIMU.getTopic().c_str());

}

/**
 * \brief This will read in config values from the launch file for our sigmas
 */
void SimulationLoader::setup_config(ros::NodeHandle& nh) {

    // Rate of the sensors
    nh.param<int>("imurate", imurate, 100);
    nh.param<int>("camrate", camrate, 10);

    // Print it all out for our user
    ROS_INFO("======================================");
    ROS_INFO("=====      RATE PARAMETERS       =====");
    ROS_INFO("======================================");
    ROS_INFO("\timurate = %d",imurate);
    ROS_INFO("\tcamrate = %d",camrate);

}



/**
 * \brief Will loop through our data, and publish it in realtime
 */
void SimulationLoader::execute_publishing(void (*f_truth)(geometry_msgs::PoseStamped::Ptr),
                                          void (*f_imu)(sensor_msgs::Imu::Ptr),
                                          void (*f_uv)(cpi_comm::CameraMeasurement::Ptr)) {

    // Our total bag time
    double timetotal = poses_imu.at(poses_imu.size()-1).header.stamp.toSec()-poses_imu.at(0).header.stamp.toSec();

    // Path count, used to skip publishing on all imu messages
    size_t pathct = 0;

    // Debug info
    ROS_INFO("Starting publishing simulated data!!");
    ROS_INFO("Looping at max rate of %d",std::max(imurate,camrate));
    ROS_INFO("Total runtime is %.2f seconds\n",timetotal);

    // Our loop rate should be faster then our fastest message type
    ros::Rate loop_rate(std::max(imurate,camrate));

    // Loop till we have used up all measurements
    // Also check to make sure that ROS does not want us to stop
    while(ros::ok() && index_pi < poses_imu.size() && index_mi < measurements_imu.size() && index_mc < measurements_camera.size()) {

        // Get what our timestamps are (note we set it to infinity if we are done with that data vector)
        double timepi = (index_pi < poses_imu.size())? poses_imu.at(index_pi).header.stamp.toSec() : std::numeric_limits<double>::max();
        double timemi = (index_mi < measurements_imu.size())? measurements_imu.at(index_mi).header.stamp.toSec() : std::numeric_limits<double>::max();
        double timeml = (index_mc < measurements_camera.size())? measurements_camera.at(index_mc).header.stamp.toSec() : std::numeric_limits<double>::max();

        // First calculate the current min time
        double timecurr = std::min(timepi,std::min(timemi,timeml));

        // Check to see if the IMU POSE is the min
        if(timepi <= timecurr) {

            // Send to our estimator
            geometry_msgs::PoseStamped::Ptr meas(new geometry_msgs::PoseStamped(poses_imu.at(index_pi)));
            (*f_truth)(meas);

            // Publish normal pose
            pubPoseIMU.publish(poses_imu.at(index_pi));

            // Publish TF message
            //tf::StampedTransform tfPose;
            //tfPose.stamp_ = poses_imu.at(index_pi).header.stamp;
            //tfPose.frame_id_ = "global";
            //tfPose.child_frame_id_ = "imu";
            //tf::Quaternion quat(poses_imu.at(index_pi).pose.orientation.x,poses_imu.at(index_pi).pose.orientation.y,poses_imu.at(index_pi).pose.orientation.z,poses_imu.at(index_pi).pose.orientation.w);
            //tfPose.setRotation(quat);
            //tf::Vector3 orig(poses_imu.at(index_pi).pose.position.x,poses_imu.at(index_pi).pose.position.y,poses_imu.at(index_pi).pose.position.z);
            //tfPose.setOrigin(orig);
            //mTfBr->sendTransform(tfPose);

            // Publish the path
            pathct = (pathct+1)%camrate;
            if(pathct == 0) {
                posespath_imu.push_back(poses_imu.at(index_pi));
                nav_msgs::Path patharr;
                patharr.header.stamp = poses_imu.at(index_pi).header.stamp;
                patharr.header.seq = poses_seq_imu++;
                patharr.header.frame_id = "global";
                patharr.poses = posespath_imu;
                pubPathIMU.publish(patharr);
            }

            // Move forward in time
            index_pi++;
        }

        // Check to see if the IMU MEASUREMENT is the min
        if(timemi <= timecurr) {
            // Send to our estimator
            sensor_msgs::Imu::Ptr meas(new sensor_msgs::Imu(measurements_imu.at(index_mi)));
            (*f_imu)(meas);
            index_mi++;
        }

        // Check to see if the CAMERA MEASUREMENT is the min
        if(timeml <= timecurr) {
            // Send to our estimator
            cpi_comm::CameraMeasurement::Ptr meas(new cpi_comm::CameraMeasurement(measurements_camera.at(index_mc)));
            (*f_uv)(meas);
            // Move forward in time
            index_mc++;
        }

        // Get what our "new" timestamps are
        timepi = (index_pi < poses_imu.size())? poses_imu.at(index_pi).header.stamp.toSec() : std::numeric_limits<double>::max();
        timemi = (index_mi < measurements_imu.size())? measurements_imu.at(index_mi).header.stamp.toSec() : std::numeric_limits<double>::max();
        timeml = (index_mc < measurements_camera.size())? measurements_camera.at(index_mc).header.stamp.toSec() : std::numeric_limits<double>::max();

        // Finally calculate what the "next" timestep will be
        double timenext = std::min(timepi,std::min(timemi,timeml));

        // We are done if we have an infinite time!
        if(timenext == std::numeric_limits<double>::max())
            break;

        // Debug print
        //ROS_INFO("SLEEP DT = %.6f (seconds)",timenext-timecurr);
        //double timeelapsed = timecurr-poses_imu.at(0).header.stamp.toSec();
        //printf("\r [RUNNING]  Simulation Time: %13.6f   Duration: %.6f / %.6f     \r", timecurr, timeelapsed, timetotal);
        //fflush(stdout);

        // Make sure ROS publishes the messages
        ros::spinOnce();

        // Thus we should sleep the amount between our current and the next timestamp
        //ros::Duration(timenext-timecurr).sleep();
        //usleep((uint)(1e6*(timenext-timecurr)));
        //loop_rate.sleep();

    }

}



