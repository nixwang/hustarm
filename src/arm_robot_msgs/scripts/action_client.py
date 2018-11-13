#! /usr/bin/env python

from __future__ import print_function
import math

import rospy
# Brings in the SimpleActionClient
import actionlib

import trajectory_msgs.msg
import control_msgs.msg

from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
)

# Brings in the messages used by the fibonacci action, including the
# goal_right message and the result message.
import actionlib_tutorials.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client_right = actionlib.SimpleActionClient('/arm/right/follow_joint_trajectory', FollowJointTrajectoryAction)
    client_left = actionlib.SimpleActionClient('/arm/left/follow_joint_trajectory', FollowJointTrajectoryAction)

    client_right.wait_for_server()
    # client_left.wait_for_server()
    print('server on')

    # Creates a goal_right to send to the action server.
    goal_right = FollowJointTrajectoryGoal()
    # goal_left = FollowJointTrajectoryGoal()
    goal_right.goal_time_tolerance = rospy.Time.from_sec(0)
    # goal_left.goal_time_tolerance = rospy.Time.from_sec(0)
    for i in range(7):
        goal_right.trajectory.joint_names.append("right_joint" + str(i + 1))

        goal_tol = control_msgs.msg.JointTolerance()
        goal_tol.name = "right_joint" + str(i + 1)
        goal_tol.position = 0.1 / 180 * math.pi
        goal_right.goal_tolerance.append(goal_tol)

        path_tol = control_msgs.msg.JointTolerance()
        path_tol.name = "right_joint" + str(i + 1)
        path_tol.position = 720 / 180 * math.pi
        goal_right.path_tolerance.append(path_tol)
    # for i in range(7):
    #     goal_left.trajectory.joint_names.append("left_joint" + str(i + 1))

    #     goal_tol = control_msgs.msg.JointTolerance()
    #     goal_tol.name = "left_joint" + str(i + 1)
    #     goal_tol.position = 0.1 / 180 * math.pi
    #     goal_left.goal_tolerance.append(goal_tol)

    #     path_tol = control_msgs.msg.JointTolerance()
    #     path_tol.name = "left_joint" + str(i + 1)
    #     path_tol.position = 180 / 180 * math.pi
    #     goal_left.path_tolerance.append(path_tol)

    dur = float(10)
    points_length = int(100)
    pos = float(360)

    for t in range(points_length + 1):
        percent = float(t) / points_length
        p = trajectory_msgs.msg.JointTrajectoryPoint()

        pp = (1 - percent) / 0.5 * pos / 180 * math.pi
        if percent <= 0.5:
            pp = percent / 0.5 * pos / 180 * math.pi

        p.positions = [0, 0, 0, 0, pp, 0, 0]
        p.time_from_start = rospy.Time.from_sec(percent * dur)
        goal_right.trajectory.points.append(p)
        # goal_left.trajectory.points.append(p)

    # Sends the goal_right to the action server.
    client_right.send_goal(goal_right)
    # client_left.send_goal(goal_left)
    print('goal_right sent')

    # Waits for the server to finish performing the action.
    client_right.wait_for_result()
    # client_left.wait_for_result()
    print('got result')

    # Prints out the result of executing the action
    return (client_right.get_result(), client_left.get_result())  # A FibonacciResult

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('action_client_py')
    result = fibonacci_client()
    print(result)
