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


#ifndef PARSELINES_H
#define PARSELINES_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>


#include "cpi_comm/FeatureMeasurement.h"
#include "cpi_comm/CameraMeasurement.h"

#include "utils/quat_ops.h"


class SimParser {
public:

    /**
     * \brief Given a string line, this will create a ROS pose object from the simulated data format
     */
    static geometry_msgs::PoseStamped parsePoseLine(ros::Time basetime, std::string frame, std::string line) {
        // Temp variables
        double timestamp = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 0.0;
        // Loop variables
        int i = 0;
        std::istringstream s(line);
        std::string field;
        // Loop through this line
        while (getline(s,field,' ')) {
            // Skip if empty
            if(field.empty())
                continue;
            // column 1: qx rotation
            if(i==0) {
                qx = std::atof(field.c_str());
            }
            // column 2: qy rotation
            else if(i==1) {
                qy = std::atof(field.c_str());
            }
            // column 3: qz rotation
            else if(i==2) {
                qz = std::atof(field.c_str());
            }
            // column 4: qw rotation
            else if(i==3) {
                qw = std::atof(field.c_str());
            }
            // column 5: x position
            else if(i==4) {
                x = std::atof(field.c_str());
            }
            // column 6: y position
            else if(i==5) {
                y = std::atof(field.c_str());
            }
            // column 7: z position
            else if(i==6) {
                z = std::atof(field.c_str());
            }
            // column 9: timestamp in ms
            else if(i==8) {
                timestamp = 1e-3*std::atof(field.c_str());
            }
            i++;
        }
        // Create the pose!
        geometry_msgs::PoseStamped pose;
        //pose.header.stamp = ros::Time(basetime.toSec()+timestamp);
        pose.header.stamp = ros::Time(timestamp);
        pose.header.frame_id = frame;
        // Pose position
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        // Return!
        return pose;
    }



    /**
     * \brief Given a string line, this will create an IMU measurement from the simulated data format
     */
    static sensor_msgs::Imu parseImuMeasurementLine(ros::Time basetime, std::string frame, std::string line) {
        // Temp variables
        double timestamp = 0.0;
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        // Loop variables
        int i = 0;
        std::istringstream s(line);
        std::string field;
        // Loop through this line
        while (getline(s,field,' ')) {
            // Skip if empty
            if(field.empty())
                continue;
            // column 1: x angular
            if(i==0) {
                wx = std::atof(field.c_str());
            }
            // column 2: y angular
            else if(i==1) {
                wy = std::atof(field.c_str());
            }
            // column 3: z angular
            else if(i==2) {
                wz = std::atof(field.c_str());
            }
            // column 4: x acceleration
            else if(i==3) {
                ax = std::atof(field.c_str());
            }
            // column 5: y acceleration
            else if(i==4) {
                ay = std::atof(field.c_str());
            }
            // column 6: z acceleration
            else if(i==5) {
                az = std::atof(field.c_str());
            }
            // column 8: timestamp in ms
            else if(i==7) {
                timestamp = 1e-3*std::atof(field.c_str());
            }
            i++;
        }
        // Create the measurement!
        sensor_msgs::Imu meas;
        //meas.header.stamp = ros::Time(basetime.toSec()+timestamp);
        meas.header.stamp = ros::Time(timestamp);
        meas.header.frame_id = frame;
        // Linear acceleration
        meas.linear_acceleration.x = ax;
        meas.linear_acceleration.y = ay;
        meas.linear_acceleration.z = az;
        // Angular velocity
        meas.angular_velocity.x = wx;
        meas.angular_velocity.y = wy;
        meas.angular_velocity.z = wz;
        // Return!
        return meas;
    }


    /**
     * \brief Given a string line, this will create a measurement array from the simulated data format
     */
    static cpi_comm::CameraMeasurement parseCameraMeasurementLine(ros::Time basetime, std::string frame, std::string lineMEAS0,
                                                                     std::string lineIDS0, std::string lineMEAS1, std::string lineIDS1) {
        // Temp variables
        double timestamp = 0.0;

        std::vector<uint> idsleft;
        std::vector<uint> idsright;
        std::vector<Eigen::Vector2d> measleft;
        std::vector<Eigen::Vector2d> measright;

        // Loop variables
        std::istringstream sMEAS0(lineMEAS0);
        std::istringstream sMEAS1(lineMEAS1);
        std::istringstream sIDS0(lineIDS0);
        std::istringstream sIDS1(lineIDS1);
        std::string fieldMEAS;
        std::string fieldIDS;

        // Left IDs
        while (getline(sIDS0,fieldIDS,' ')) {
            if(fieldIDS.empty()) continue;
            idsleft.push_back((uint)std::atoi(fieldIDS.c_str()));
        }

        // Right IDs
        while (getline(sIDS1,fieldIDS,' ')) {
            if(fieldIDS.empty()) continue;
            idsright.push_back((uint)std::atoi(fieldIDS.c_str()));
        }

        // Temp vectors (for some reason the simulation files give all Us then al Vs)
        std::vector<double> tempall;

        // Get LEFT the UV coordinates
        while (getline(sMEAS0,fieldMEAS,' ')) {
            if(fieldMEAS.empty()) continue;
            tempall.push_back(std::atof(fieldMEAS.c_str()));
        }

        // The last data is the timestamp of these UV coords
        timestamp = 1e-3*tempall.at(tempall.size()-1);
        tempall.pop_back();

        // Convert LEFT to correct UV coordinates
        for(size_t i=0; i<tempall.size()/2; i++) {
            Eigen::Vector2d uv;
            uv << tempall.at(i),tempall.at(i+tempall.size()/2);
            measleft.push_back(uv);
        }

        // Clear the vector
        tempall.clear();

        // Get RIGHT the UV coordinates
        while (getline(sMEAS1,fieldMEAS,' ')) {
            if(fieldMEAS.empty()) continue;
            tempall.push_back(std::atof(fieldMEAS.c_str()));
        }

        // The last data is the timestamp of these UV coords
        timestamp = 1e-3*tempall.at(tempall.size()-1);
        tempall.pop_back();

        // Convert RIGHT to correct UV coordinates
        for(size_t i=0; i<tempall.size()/2; i++) {
            Eigen::Vector2d uv;
            uv << tempall.at(i),tempall.at(i+tempall.size()/2);
            measright.push_back(uv);
        }

        // Print out the amount
        //ROS_INFO("[CAM-MEAS]: timestamp = %.3f", timestamp);
        //ROS_INFO("[CAM-MEAS]: %d ids left | %d ids right", (int)idsleft.size(), (int)idsright.size());
        //ROS_INFO("[CAM-MEAS]: %d meas left | %d meas right", (int)measleft.size(), (int)measright.size());

        // Assert they are all equal sizes
        assert(idsleft.size() == measleft.size());
        assert(idsright.size() == measright.size());

        // Our final measurement
        cpi_comm::CameraMeasurement measurement;
        //measurement.header.stamp = ros::Time(basetime.toSec()+timestamp);
        measurement.header.stamp = ros::Time(timestamp);
        measurement.header.frame_id = frame;

        // Left UV features
        for(size_t i=0;  i<idsleft.size(); i++) {
            cpi_comm::FeatureMeasurement feat;
            feat.id = idsleft.at(i);
            feat.u = measleft.at(i)(0);
            feat.v = measleft.at(i)(1);
            measurement.features_left.push_back(feat);
        }

        // Right UV features
        for(size_t i=0;  i<idsright.size(); i++) {
            cpi_comm::FeatureMeasurement feat;
            feat.id = idsright.at(i);
            feat.u = measright.at(i)(0);
            feat.v = measright.at(i)(1);
            measurement.features_right.push_back(feat);
        }

        // Return!
        return measurement;
    }



};




#endif  //#ifndef PARSELINES_H
