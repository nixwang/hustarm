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

#include "bmx055_bridge.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "msg_bridge");

    ros::NodeHandle nh("msg_bridge");

    hustac::BMX055Bridge imu_bridge_camera(nh, "/camera/imu/");

    ros::spin();

    return 0;
}

