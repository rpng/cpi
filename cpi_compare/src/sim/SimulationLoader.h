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


#ifndef SIMULATIONLOADER_H
#define SIMULATIONLOADER_H

#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <random>
#include <string>

#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>


#include "SimParser.h"
#include "cpi_comm/CameraMeasurement.h"

using namespace std;


class SimulationLoader {
public:

    /**
     * Default constructor
     * Will load all our data from file
     */
    SimulationLoader(std::string path, ros::NodeHandle nh, ros::NodeHandle nhprivate) {
        this->path = path;
        load_files(path);
        setup_publishers(nh);
        setup_config(nhprivate);
    }


    /**
     * Function will run the simulation
     */
    void execute_publishing(void (*f_truth)(geometry_msgs::PoseStamped::Ptr),
                            void (*f_imu)(sensor_msgs::Imu::Ptr),
                            void (*f_uv)(cpi_comm::CameraMeasurement::Ptr));

private:

    // Functions
    void load_files(std::string path);
    void setup_publishers(ros::NodeHandle nh);
    void setup_config(ros::NodeHandle& nh);

    /// Directory of files
    std::string path;

    // Simulation data from file
    int imurate;
    int camrate;
    std::vector<geometry_msgs::PoseStamped> poses_imu;
    std::vector<geometry_msgs::PoseStamped> poses_cam;
    std::vector<sensor_msgs::Imu> measurements_imu;
    std::vector<cpi_comm::CameraMeasurement> measurements_camera;


    // Current index in each data vector
    size_t index_pi = 0;
    size_t index_mi = 0;
    size_t index_mc = 0;


    // Publishers
    tf::TransformBroadcaster* mTfBr;
    ros::Publisher pubPoseIMU;
    ros::Publisher pubPoseCAM0;
    ros::Publisher pubPathIMU;

    // Path variables needed
    unsigned int poses_seq_imu = 0;
    vector<geometry_msgs::PoseStamped> posespath_imu;


};



#endif /* SIMULATIONLOADER_H */
