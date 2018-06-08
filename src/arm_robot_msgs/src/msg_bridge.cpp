/*
 * msg_bridge.cpp
 *
 *  Created on: 2018年6月8日
 *      Author: hustac
 */

#include <string>
#include <vector>
#include <iostream>
#include <functional>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "arm_robot_msgs/Imu.h"
#include "arm_robot_msgs/MagneticField.h"

static void imuCallback(const arm_robot_msgs::Imu::ConstPtr& raw_msg,
        ros::Publisher& pub) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = raw_msg->header;
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity = raw_msg->angular_velocity;
    imu_msg.linear_acceleration = raw_msg->linear_acceleration;
    pub.publish(imu_msg);
}

static void magCallback(const arm_robot_msgs::MagneticField::ConstPtr& raw_msg,
        ros::Publisher& pub) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = raw_msg->header;
    mag_msg.magnetic_field = raw_msg->magnetic_field;
    pub.publish(mag_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "msg_bridge");

    ros::NodeHandle n;

    std::vector<ros::Subscriber> subs;
    subs.push_back(
            n.subscribe<arm_robot_msgs::Imu>("imu_camera_raw", 100,
                    std::bind(imuCallback, std::placeholders::_1,
                            n.advertise<sensor_msgs::Imu>("imu_camera", 100))));
    subs.push_back(
            n.subscribe<arm_robot_msgs::MagneticField>("mag_camera_raw", 100,
                    std::bind(magCallback, std::placeholders::_1,
                            n.advertise<sensor_msgs::MagneticField>("mag_camera", 100))));

    ros::spin();

    return 0;
}

