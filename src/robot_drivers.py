#!/usr/bin/python
from __future__ import print_function 
import roslib;roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *


# This class provides services about UR5 robot
# as move_to method and so on.
class RobotTrajectory(object):

    def __init__(self, joint_name, node_name, subscriber_topic):
        # Names of the joints : use for kinetmatic
        assert isinstance(joint_name, list)
        self.JOINT_NAMES = joint_name
        
        # Subscriber
        self.subscriber_topic = subscriber_topic
        self.sub = rospy.Subscriber(self.subscriber_topic, JointTrajectory, self.callback)

        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        
        # Client from action lib
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        self.client.wait_for_server()
        print("Connected to the server")

        # Duration
        self.d = 0
    
    def run(self):
        """
        Launch the infinite loop
        """
        
        rospy.spin()
    
    def callback(self, msg):
        """
        Callback for the subscriber
        Receive a JointTrajectory and send it to the robot
        """
        self.move_joint(msg)
    
    def move_joint(self, joint_trajectory):
        """
        Move the robot in the joint space to a given point
        
        :param joint_trajetory: ROS JointTrajectory msg
        """
        # Trajectory
        g = FollowJointTrajectoryGoal()
        g.trajectory = joint_trajectory
        g.trajectory.joint_names = self.JOINT_NAMES
        self.d += g.trajectory.points[-1].time_from_start.secs
        # Start the trajectory from now
        g.trajectory.header.stamp = rospy.Time.now()
    
        # Send the trajectory to the client
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
