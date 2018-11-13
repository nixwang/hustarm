#!/usr/bin/env python
from __future__ import print_function

import rosbag
import sys
from std_msgs.msg import Int32, String


def print_help():
    print("usage: rosrun arm_robot_msgs bag_to_txyz.py path_to_bag_file topic_imu output_file_accel output_file_gyro")


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print_help()
    else:
        bag_path = sys.argv[1]
        topic_imu = sys.argv[2]
        output_path_accel = sys.argv[3]
        output_path_gyro = sys.argv[4]
        print("Bag file:", bag_path)
        print("Topic imu:", topic_imu)
        print("Output accel file:", output_path_accel)
        print("Output gyro file:", output_path_gyro)

        bag = rosbag.Bag(bag_path, 'r')
        output_file_accel = open(output_path_accel, 'w')
        output_file_gyro = open(output_path_gyro, 'w')

        first_time = -1
        for topic, msg, t in bag.read_messages(topics=[topic_imu]):
            if first_time < 0:
                first_time = msg.header.stamp.to_sec()
            output_file_accel.write('%.6f\t' % (msg.header.stamp.to_sec() - first_time))
            output_file_accel.write('%.6e\t' % msg.linear_acceleration.x)
            output_file_accel.write('%.6e\t' % msg.linear_acceleration.y)
            output_file_accel.write('%.6e\r\n' % msg.linear_acceleration.z)
            output_file_gyro.write('%.6f\t' % (msg.header.stamp.to_sec() - first_time))
            output_file_gyro.write('%.6e\t' % (msg.angular_velocity.x))
            output_file_gyro.write('%.6e\t' % (msg.angular_velocity.y))
            output_file_gyro.write('%.6e\r\n' % (msg.angular_velocity.z))

        bag.close()
        output_file_accel.close()
        output_file_gyro.close()
        
        
