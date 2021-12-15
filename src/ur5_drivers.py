#!/usr/bin/python
from __future__ import print_function 
import roslib;roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from robot_drivers import RobotTrajectory


class UR5TrajectoryPlanner(RobotTrajectory):

    def __init__(self):

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        super(UR5TrajectoryPlanner, self).__init__(joint_anmes,
                                                   'robot_ur5',
                                                   'ur5_trajectory_goal')


if __name__ == "__main__":

    ur5_trajectory = UR5TrajectoryPlanner()

    ur5_trajectory.run()
