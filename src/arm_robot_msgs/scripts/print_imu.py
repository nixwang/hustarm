#!/usr/bin/env python

from math import sqrt

import rospy
from sensor_msgs.msg import Imu as StdImu

from arm_robot_msgs.msg import Imu as SimpleImu

std_imu_msg = None
simple_imu_msg = None

def callback_StdImu(data):
    global std_imu_msg
    std_imu_msg = data


def callback_SimpleImu(data):
    global simple_imu_msg
    simple_imu_msg = data
    
    
def callback_timer(event):
    if simple_imu_msg:
        data = simple_imu_msg
        rospy.loginfo("Raw accel:        %f %f %f (%f)",
                      data.linear_acceleration.x,
                      data.linear_acceleration.y,
                      data.linear_acceleration.z,
                      sqrt(data.linear_acceleration.x * data.linear_acceleration.x + 
                           data.linear_acceleration.y * data.linear_acceleration.y + 
                           data.linear_acceleration.z * data.linear_acceleration.z))
    if std_imu_msg:
        data = std_imu_msg
        rospy.loginfo("Calibrated accel: %f %f %f (%f)",
                      data.linear_acceleration.x,
                      data.linear_acceleration.y,
                      data.linear_acceleration.z,
                      sqrt(data.linear_acceleration.x * data.linear_acceleration.x + 
                           data.linear_acceleration.y * data.linear_acceleration.y + 
                           data.linear_acceleration.z * data.linear_acceleration.z))

    
def print_imu():
    rospy.init_node('print_imu', anonymous=True)

    global timer_StdImu, timer_SimpleImu

    rospy.Subscriber("/camera/imu_raw", SimpleImu, callback_SimpleImu)
    rospy.Subscriber("/camera/imu", StdImu, callback_StdImu)
    
    timer = rospy.Timer(rospy.Duration(1), callback_timer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    print_imu()
