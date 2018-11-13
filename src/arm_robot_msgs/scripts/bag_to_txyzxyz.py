#!/usr/bin/env python
from __future__ import print_function

import rosbag
import sys
from std_msgs.msg import Int32, String


def print_help():
    print("usage: rosrun arm_robot_msgs bag_to_txyz.py path_to_bag_file topic_imu output_file")


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print_help()
    else:
        bag_path = sys.argv[1]
        topic_imu = sys.argv[2]
        output_path = sys.argv[3]
        print("Bag file:", bag_path)
        print("Topic imu:", topic_imu)
        print("Output file:", output_path)

        bag = rosbag.Bag(bag_path, 'r')
        output_file = open(output_path, 'w')

        last_time = 0
        for topic, msg, t in bag.read_messages(topics=[topic_imu]):
            # t = msg.header.stamp.to_sec()
            # if t - last_time < 0.01:
            #     continue
            # last_time = t
            output_file.write('%.6f,' % msg.header.stamp.to_sec())
            output_file.write('%.6e,' % msg.linear_acceleration.x)
            output_file.write('%.6e,' % msg.linear_acceleration.y)
            output_file.write('%.6e,' % msg.linear_acceleration.z)
            output_file.write('%.6e,' % msg.angular_velocity.x)
            output_file.write('%.6e,' % msg.angular_velocity.y)
            output_file.write('%.6e\r\n' % msg.angular_velocity.z)

        bag.close()
