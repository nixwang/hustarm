/*
 * bag_to_csv.cpp
 *
 *  Created on: 2018年6月16日
 *      Author: hustac
 */

#include <string>
#include <iostream>
#include <ios>
#include <fstream>
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <arm_robot_msgs/Imu.h>
#include <sensor_msgs/Imu.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

static void print_help() {
    std::cout
            << "usage: rosrun arm_robot_msgs bag_to_csv path_to_bag_file topic_imu output_file" << std::endl;
}


#define write_imu(output_file, p_msg) do {\
    output_file << std::fixed;\
    output_file << p_msg->header.stamp.toSec() << ',';\
    output_file << std::scientific;\
    output_file << p_msg->linear_acceleration.x << ',';\
    output_file << p_msg->linear_acceleration.y << ',';\
    output_file << p_msg->linear_acceleration.z << ',';\
    output_file << p_msg->angular_velocity.x << ',';\
    output_file << p_msg->angular_velocity.y << ',';\
    output_file << p_msg->angular_velocity.z << std::endl;\
    output_file << std::defaultfloat;\
} while (0)

int main(int argc, char **argv) {
    if (argc < 4) {
        print_help();
        return -1;
    }

    std::string bag_path = argv[1];
    std::string topic_imu = argv[2];
    std::string output_path = argv[3];
    std::cout << "Bag file: " << bag_path << std::endl;
    std::cout << "Topic imu: " << topic_imu << std::endl;
    std::cout << "Output file: " << output_path << std::endl;

    std::ofstream output_file(output_path);

    std::cout << "Loading bag file..." << std::flush;
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(topic_imu);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::cout << "OK" << std::endl;

    auto last_print = std::chrono::steady_clock::now();
    uint64_t count_imu = 0;
    std::cout << "Message count: " << count_imu << std::flush;

    foreach(rosbag::MessageInstance const msg, view) {
        arm_robot_msgs::Imu::ConstPtr msg_simple_imu =
                msg.instantiate<arm_robot_msgs::Imu>();
        if (msg_simple_imu != NULL) {
            write_imu(output_file, msg_simple_imu);
            count_imu++;
        } else {
            sensor_msgs::Imu::ConstPtr msg_imu =
                    msg.instantiate<sensor_msgs::Imu>();
            if (msg_imu != NULL) {
                write_imu(output_file, msg_imu);
                count_imu++;
            }
        }

        if (std::chrono::steady_clock::now() - last_print >= std::chrono::seconds(1)) {
            last_print = std::chrono::steady_clock::now();
            std::cout << '\r' << "Message count: " << count_imu << std::flush;
        }
    }
    std::cout << std::endl;

    std::cout << "Finished " << count_imu << std::endl;

    bag.close();
    return 0;
}

